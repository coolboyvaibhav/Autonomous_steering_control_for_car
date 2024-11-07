# %%  https://www.youtube.com/watch?v=s2QEhSpmhVA--- for lane detection 
#https://www.youtube.com/watch?v=pJeQcnn-sxI -- see
import torch

# load model
model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True)

# %%
