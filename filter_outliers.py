import numpy as np 

def removeOutliers(x, outlierConstant=1.5):
    """
    Remove the outliers of a numpy array with the IQR method
    """
    # a = np.array(x)
    upper_quartile = np.percentile(x, 75)
    lower_quartile = np.percentile(x, 25)
    IQR = (upper_quartile - lower_quartile) * outlierConstant 
    quartileSet = (lower_quartile - IQR, upper_quartile + IQR)
    
    result = x[np.where((x >= quartileSet[0]) & (x <= quartileSet[1]))]
    
    return result