#!/usr/bin/env python3
"""壊れた (途中で切れた) bag から、速度解析に要るトピックだけをページ直読みで抜き出す。

    python3 recover_small.py <db3> <out.pkl>

録画を強制終了して sqlite3 が "database disk image is malformed" で開けない
bag 用 (extract_small.py が使えないとき)。Jetson ホストで実行して、小さい
pkl だけを PC へ送る。ROS も numpy も不要。

recover.py との違い:
  - ファイルを mmap で読むので、数 GB の bag でもメモリをほとんど使わない
  - TOPICS 以外のメッセージは本体 (overflow ページ) を読まずに捨てる
出力の形式は recover.py と同じなので、そのまま decode.py に渡せる。
"""
import mmap
import pickle
import struct
import sys
from collections import Counter

TOPICS = ('/cmd_vel_raw', '/cmd_vel', '/odom', '/odometry/filtered', '/imu',
          '/motor/pwm_count', '/motor/cmd_normalized', '/motor/steer_angle',
          '/motor/speed_ctrl', '/rosout', '/amcl_pose')

path, out = sys.argv[1], sys.argv[2]
f = open(path, 'rb')
buf = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
PS = int.from_bytes(buf[16:18], 'big')
if PS == 1:
    PS = 65536
U = PS - buf[20]
NP = len(buf) // PS
X = U - 35
M = ((U - 12) * 32 // 255) - 23


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


def cell(pg, off):
    """(rowid, 全長 P, ページ内の部分, 先頭 overflow ページ番号)"""
    P, i = varint(pg, off)
    rowid, i = varint(pg, i)
    if P <= X:
        return rowid, P, pg[i:i + P], 0
    K = M + ((P - M) % (U - 4))
    local = K if K <= X else M
    return rowid, P, pg[i:i + local], int.from_bytes(pg[i + local:i + local + 4], 'big')


def follow(data, P, ov):
    data = bytearray(data)
    while ov and len(data) < P:
        if ov > NP:
            return None
        base = (ov - 1) * PS
        ov = int.from_bytes(buf[base:base + 4], 'big')
        data += buf[base + 4:base + U]
    return bytes(data[:P])


def header(p):
    hs, i = varint(p, 0)
    types = []
    while i < hs:
        t, i = varint(p, i)
        types.append(t)
    return types, hs


def record(p):
    types, j = header(p)
    vals = []
    for t in types:
        if t == 0:
            vals.append(None)
        elif 1 <= t <= 6:
            n = [1, 2, 3, 4, 6, 8][t - 1]
            vals.append(int.from_bytes(p[j:j + n], 'big', signed=True)); j += n
        elif t == 7:
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


def leaf_cells():
    for n in range(1, NP + 1):
        base = (n - 1) * PS
        h = 100 if n == 1 else 0
        if buf[base + h] != 0x0D:
            continue
        pg = buf[base:base + PS]
        nc = int.from_bytes(pg[h + 3:h + 5], 'big')
        for c in range(nc):
            off = int.from_bytes(pg[h + 8 + 2 * c:h + 10 + 2 * c], 'big')
            try:
                yield cell(pg, off)
            except Exception:
                continue


# 1 周目: topics テーブルの行 (小さいので overflow しない)
topics = {}
for rowid, P, loc, ov in leaf_cells():
    if ov:
        continue
    try:
        v = record(loc)
    except Exception:
        continue
    if len(v) >= 5 and isinstance(v[1], str) and isinstance(v[2], str) and '/' in v[2]:
        topics[rowid] = (v[1], v[2])
want = {k for k, (name, _) in topics.items() if name in TOPICS}
print('topics', len(topics), '-> keep', sorted(topics[k][0] for k in want))

# 2 周目: messages (id, topic_id, timestamp, data)。topic_id はページ内の
# 先頭部分で読めるので、要らないものは overflow を辿らずに捨てる。
msgs, stats = {}, Counter()
for rowid, P, loc, ov in leaf_cells():
    try:
        types, j = header(loc)
        if len(types) != 4 or not (1 <= types[1] <= 9) or types[1] == 7 \
                or types[3] < 12 or types[3] % 2:
            continue
        if types[0] != 0:   # id は rowid の別名なので NULL のはず
            continue
        if types[1] >= 8:   # 整数 0 / 1 は値を持たない型 8 / 9 で入る
            tid = types[1] - 8
        else:
            n = [1, 2, 3, 4, 6, 8][types[1] - 1]
            tid = int.from_bytes(loc[j:j + n], 'big', signed=True)
        if tid not in want:
            continue
        p = follow(loc, P, ov) if ov else bytes(loc)
        if p is None:
            stats['overflow_lost'] += 1
            continue
        v = record(p)
    except Exception:
        stats['bad'] += 1
        continue
    if isinstance(v[1], int) and isinstance(v[2], int) and isinstance(v[3], bytes):
        msgs[rowid] = (v[1], v[2], v[3])

print('pages', NP, 'page size', PS, dict(stats))
cnt = Counter(t for t, _, _ in msgs.values())
for tid, k in sorted(cnt.items()):
    ts = [s for t, s, _ in msgs.values() if t == tid]
    print(tid, topics.get(tid), k, '%.1fs' % ((max(ts) - min(ts)) / 1e9))
pickle.dump({'topics': topics, 'msgs': msgs}, open(out, 'wb'))
