import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob

files = glob.glob("val_*.png")
fig, axs = plt.subplots(3, 2, figsize=(20, 15))

for i, f in enumerate(sorted(files)):
    if "summary" in f: continue
    img = mpimg.imread(f)
    ax = axs[i // 2, i % 2]
    ax.imshow(img)
    ax.axis('off')

plt.tight_layout()
plt.savefig("val_summary.png")
