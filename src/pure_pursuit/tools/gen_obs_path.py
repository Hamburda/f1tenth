# Folosire: python3 gen_obs_path.py <folder harti> <iesire.csv> 0.145 0.3 3000 "-0.29,-0.15,0,0.15,0.29"
# Genereaza un traseu care ocoleste obstacolele de pe levine_obs:
# pentru fiecare punct al liniei de centru alege un offset lateral (pe normala),
# prin programare dinamica: penalizare pentru distanta mica fata de obstacole + netezime.
import sys, numpy as np, yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev

maps, out = sys.argv[1], sys.argv[2]
m = yaml.safe_load(open(f'{maps}/levine_obs.yaml'))
img = np.array(Image.open(f'{maps}/' + m['image']).convert('L'))
res = m['resolution']; ox, oy = m['origin'][:2]; H = img.shape[0]
clear = ndimage.distance_transform_edt(img >= 250) * res   # doar albul e liber (gri = necunoscut)

def clearance(p):
    c = ((p[..., 0] - ox) / res).astype(int); r = (H - 1 - (p[..., 1] - oy) / res).astype(int)
    return clear[r, c]

w = np.loadtxt(f'{maps}/levine_centerline.csv', delimiter=',', comments='#', usecols=(0, 1))
N = len(w)
t = np.roll(w, -1, 0) - np.roll(w, 1, 0); t /= np.linalg.norm(t, axis=1, keepdims=True)
n = np.stack([-t[:, 1], t[:, 0]], 1)                     # normala spre stanga
offs = np.arange(-2.0, 2.0001, 0.05); K = len(offs)
P = w[:, None, :] + offs[None, :, None] * n[:, None, :]  # (N, K, 2)
# traseul e planificat pentru centrul caroseriei (0.58 x 0.31 m); verificam si +-0.15 m in lungul ei
body = np.array([float(b) for b in (sys.argv[6] if len(sys.argv) > 6 else "-0.15,0,0.15").split(",")])
cl = np.min([clearance(P + b * t[:, None, :]) for b in body], axis=0)
HARD = float(sys.argv[3]) if len(sys.argv) > 3 else 0.14
SAFE = float(sys.argv[4]) if len(sys.argv) > 4 else 0.45
TW = float(sys.argv[5]) if len(sys.argv) > 5 else 200.0
node = np.where(cl < HARD, 1e6, np.maximum(0, SAFE - cl) ** 2 * 2000) + 0.02 * offs[None, :] ** 2
dO = np.abs(offs[:, None] - offs[None, :])
trans = np.where(dO > 0.08, 1e6, TW * dO ** 2)        # netezime: max 8 cm lateral per pas (~0.12 m)

# DP pe bucla: pornim de la un punct sigur (offset 0 fixat)
s = int(np.argmax(cl[:, K // 2]))
order = [(s + i) % N for i in range(N)]
cost = np.full(K, 1e9); cost[K // 2] = 0; back = []
for i in order[1:] + [s]:
    tot = cost[:, None] + trans
    b = np.argmin(tot, 0); back.append(b)
    cost = tot[b, np.arange(K)] + node[i]
k = K // 2; ks = [k]
for b in reversed(back[1:]):
    k = b[k]; ks.append(k)
ks = [K // 2] + ks[::-1][:-1]                            # stari pentru order[0..N-1]

# netezire pe offset-uri (nu pe x,y), ca sa nu taie colturile, apoi reesantionare la ~0.1 m
o = np.empty(N); o[order] = offs[ks]
o = ndimage.gaussian_filter1d(o, sigma=1.5, mode='wrap')
path = w + o[:, None] * n
closed = np.vstack([path, path[:1]])
seg = np.linalg.norm(np.diff(closed, axis=0), axis=1); sc = np.concatenate([[0], np.cumsum(seg)])
L = sc[-1]; u = np.arange(0, L, 0.1)
out_pts = np.stack([np.interp(u, sc, closed[:, 0]), np.interp(u, sc, closed[:, 1])], 1)
# odometria (si pure pursuit) urmaresc puntea spate, aflata cu 0.165 m in spatele centrului caroseriei
tg = np.roll(out_pts, -1, 0) - np.roll(out_pts, 1, 0); tg /= np.linalg.norm(tg, axis=1, keepdims=True)
center_pts = out_pts.copy(); out_pts = out_pts - 0.165 * tg
i0 = np.argmin(np.linalg.norm(out_pts - w[0], axis=1)); out_pts = np.roll(out_pts, -i0, 0)
c2 = clearance(center_pts)
print(f'points {len(out_pts)}, length {L:.1f} m, min clearance {c2.min():.2f} m, <0.4: {(c2 < 0.4).sum()}')
np.savetxt(out, out_pts, delimiter=',', fmt='%.4f',
           header='levine_obs: traseu care ocoleste obstacolele (generat din levine_centerline)\nx_m,y_m')
