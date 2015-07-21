import numpy as np
import pylab #part of matplotlib
import mahotas as mh

#%% Basic functions

dna = mh.imread('dna.jpeg')
pylab.imshow(dna) 
pylab.show()

# set from heatmap (default) to greyscale
pylab.imshow(dna)
pylab.gray()
pylab.show

print(dna.shape)
print(dna.dtype)
print(dna.max())
print(dna.min())


#from pythonvision.org/basic-tutorial/

#%% Counting nuclei - first attempt 

# Smoothing and thresholding
dnaf = mh.gaussian_filter(dna,8)
dnaf = dnaf.astype(np.uint8)
T=mh.thresholding.otsu(dnaf)
pylab.imshow(dnaf>T)
pylab.show

# Count objects
labeled,nr_objects = mh.label(dnaf>T)
print(nr_objects)
pylab.imshow(labeled)
pylab.jet()
pylab.show()


#%% Counting nuclei - improved

dnaf = mh.gaussian_filter(dna,16) # play around with sigma value for desired results
rmax = mh.regmax(dnaf) # finds regional maxima
dna_overlay = mh.overlay(dna,rmax) # gray level image with overlayed red
pylab.imshow(dna_overlay) 
pylab.show()


# Count objects
seeds, nr_nuclei = mh.label(rmax)
print(nr_nuclei)

#%% Watershed for representation of image 

# contrast strech the dist image
dnaf = dnaf.astype(np.uint8)
T = mh.thresholding.otsu(dnaf)
dist = mh.distance(dnaf>T)
dist = dist.max() - dist
dist -= dist.min()
dist = dist/float(dist.ptp()) * 255
dist= dist.astype(np.uint8)
pylab.imshow(dist)
pylab.show()

# final result
nuclei = mh.cwatershed(dist, seeds) 
pylab.imshow(nuclei)
pylab.show()

#%% Fill image using generalised Voronoi
whole = mh.segmentation.gvoronoi(nuclei)
pylab.imshow(whole)
pylab.show()

#%% Remove the nuclei that touch the border

borders = np.zeros(nuclei.shape, np.bool)
# set boarders of array to true
borders[0,:] = 1
borders[-1,:] = 1
borders[:,0] = 1
borders[:,-1] = 1
# get values of nuclei at borders (unique values - remove duplicates)
at_border = np.unique(nuclei[borders])
# set whole to 0 at the found values 
for obj in at_border:
	whole[whole==obj] = 0
