"""壊れた(途中で切れた) rosbag2 sqlite3 をページ直読みで復元する。

    python recover.py <db3> <out.pkl>

テーブル行 (topics / messages) を全ページから拾う。
"""
import pickle
import sys
from collections import Counter

path, out = sys.argv[1], sys.argv[2]
buf = open(path, 'rb').read()
PS = int.from_bytes(buf[16:18], 'big')
if PS == 1:
    PS = 65536
U = PS - buf[20]
NP = len(buf) // PS


def varint(b, i):
    v = 0
    for k in range(9):
        c = b[i + k]
        if k == 8:
            return (v << 8) | c, i + 9
        v = (v << 7) | (c & 0x7F)
        if c < 0x80:
            return v, i + k + 1
    raise ValueError


def page(n):
    return buf[(n - 1) * PS:n * PS]


def payload(pg, off):
    P, i = varint(pg, off)
    rowid, i = varint(pg, i)
    X = U - 35
    if P <= X:
        return rowid, pg[i:i + P], True
    M = ((U - 12) * 32 // 255) - 23
    K = M + ((P - M) % (U - 4))
    local = K if K <= X else M
    data = bytearray(pg[i:i + local])
    ov = int.from_bytes(pg[i + local:i + local + 4], 'big')
    while ov and len(data) < P:
        if ov > NP:
            return rowid, None, False
        op = page(ov)
        ov = int.from_bytes(op[0:4], 'big')
        data += op[4:U]
    return rowid, bytes(data[:P]), True


def record(p):
    hs, i = varint(p, 0)
    types = []
    while i < hs:
        t, i = varint(p, i)
        types.append(t)
    vals, j = [], hs
    for t in types:
        if t == 0:
            vals.append(None)
        elif 1 <= t <= 6:
            n = [1, 2, 3, 4, 6, 8][t - 1]
            vals.append(int.from_bytes(p[j:j + n], 'big', signed=True)); j += n
        elif t == 7:
            import struct
            vals.append(struct.unpack('>d', p[j:j + 8])[0]); j += 8
        elif t in (8, 9):
            vals.append(t - 8)
        elif t >= 12 and t % 2 == 0:
            n = (t - 12) // 2; vals.append(bytes(p[j:j + n])); j += n
        elif t >= 13:
            n = (t - 13) // 2; vals.append(p[j:j + n].decode('utf8', 'replace')); j += n
        else:
            raise ValueError(t)
    return vals


topics, msgs, master = {}, {}, []
stats = Counter()
for n in range(1, NP + 1):
    pg = page(n)
    h = 100 if n == 1 else 0
    if pg[h] != 0x0D:
        continue
    nc = int.from_bytes(pg[h + 3:h + 5], 'big')
    for c in range(nc):
        off = int.from_bytes(pg[h + 8 + 2 * c:h + 10 + 2 * c], 'big')
        try:
            rowid, p, ok = payload(pg, off)
            if not ok:
                stats['overflow_lost'] += 1
                continue
            v = record(p)
        except Exception:
            stats['bad'] += 1
            continue
        if n == 1 or (len(v) == 5 and v[0] in ('table', 'index')):
            master.append(v)
        elif len(v) == 4 and isinstance(v[1], int) and isinstance(v[2], int) and isinstance(v[3], bytes):
            msgs[rowid] = (v[1], v[2], v[3])
        elif len(v) >= 5 and isinstance(v[1], str) and isinstance(v[2], str) and '/' in v[2]:
            topics[rowid] = (v[1], v[2])
        else:
            stats['other%d' % len(v)] += 1

print('pages', NP, 'page size', PS, stats)
print('master', [(m[0], m[1], m[3]) for m in master])
print('topics', topics)
cnt = Counter(t for t, _, _ in msgs.values())
for tid, k in sorted(cnt.items()):
    ts = [s for t, s, _ in msgs.values() if t == tid]
    print(tid, topics.get(tid), k, '%.1fs' % ((max(ts) - min(ts)) / 1e9))
pickle.dump({'topics': topics, 'msgs': msgs}, open(out, 'wb'))
