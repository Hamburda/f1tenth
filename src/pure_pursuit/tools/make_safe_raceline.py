"""Raceline cu marjă de siguranță față de pereți.

Pentru fiecare punct al raceline-ului caută cel mai mic pas t în [0, 1] pe segmentul
raceline -> centerline (cel mai apropiat punct) astfel încât distanța la perete să fie >= margin.
t se netezește (gaussian, circular), apoi traseul se reeșantionează la pas egal.

Utilizare:
  python3 make_safe_raceline.py <harta.yaml> <centerline.csv> <raceline.csv> <iesire.csv> <margin_m>
"""
import sys
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, gaussian_filter1d

STEP = 0.2      # pasul final între puncte [m]
SIGMA = 8       # netezirea lui t, în puncte (~1.6 m)


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = yaml_path.rsplit('/', 1)[0] + '/' + meta['image']
    img = np.array(Image.open(img_path).convert('L'))
    res = meta['resolution']
    ox, oy = meta['origin'][:2]
    dist = distance_transform_edt(img >= 250) * res   # 214 = necunoscut, nu e liber

    def clearance(pts):
        c = np.clip(((pts[:, 0] - ox) / res).astype(int), 0, img.shape[1] - 1)
        r = np.clip((img.shape[0] - 1 - (pts[:, 1] - oy) / res).astype(int), 0, img.shape[0] - 1)
        return dist[r, c]
    return clearance


def resample(pts, step):
    closed = np.vstack([pts, pts[:1]])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    s_new = np.arange(0, s[-1], step)
    return np.column_stack([np.interp(s_new, s, closed[:, 0]), np.interp(s_new, s, closed[:, 1])])


def main():
    map_yaml, cl_csv, rl_csv, out_csv, margin = sys.argv[1:6]
    margin = float(margin)
    clearance = load_map(map_yaml)
    cl = np.loadtxt(cl_csv, delimiter=',', comments='#', usecols=(0, 1))
    rl = np.loadtxt(rl_csv, delimiter=';', comments='#', usecols=(1, 2))

    near = cl[np.argmin(np.linalg.norm(rl[:, None, :] - cl[None, :, :], axis=2), axis=1)]
    ts = np.linspace(0, 1, 51)
    t = np.ones(len(rl))
    for k, tk in enumerate(ts[::-1]):   # de la centerline spre raceline: ține ultimul t sigur
        ok = clearance(rl + tk * (near - rl)) >= margin
        t[ok] = tk
    # netezire: ia maximul local înainte, ca netezirea să nu reducă marja
    t = np.maximum.reduce([np.roll(t, s) for s in range(-SIGMA, SIGMA + 1)])
    t = gaussian_filter1d(t, SIGMA, mode='wrap')

    path = resample(rl + t[:, None] * (near - rl), STEP)
    path = np.column_stack([gaussian_filter1d(path[:, i], 2, mode='wrap') for i in range(2)])
    d = clearance(path)
    print(f'{len(path)} puncte, marja: min {d.min():.2f} m, mediana {np.median(d):.2f} m')
    np.savetxt(out_csv, path, delimiter=',', fmt='%.4f',
               header=f'raceline cu marja {margin} m fata de pereti (make_safe_raceline.py)\nx_m, y_m')


if __name__ == '__main__':
    main()
