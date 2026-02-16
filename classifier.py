import numpy as np
from fastai.vision.all import *
from pathlib import Path
import matplotlib.pyplot as plt 

def label_func(f): return f[0] + f[1]

path = Path.cwd() / 'data'
files = get_image_files(path)
dls = ImageDataLoaders.from_name_func(path, files, label_func, item_tfms=Resize(224))
learn = vision_learner(dls, resnet34, metrics=error_rate)
learn.fine_tune(2)
learn.show_results()
plt.show()
learn.path = Path.cwd() / 'src'
learn.export()