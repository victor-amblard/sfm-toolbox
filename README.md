Running `preprocessZip.sh` will create a directory with the following structure:

| parameters.yaml  
| --- imgs  
     |   
     --- raw  
     |    | XX.jpg  
     |    | YY.jpg  
     |  
     --- rectified  
     |    | XX_rectified.jpg   
     |    | YY_rectified.jpg  
     |  
| --- scans  

There is an example of `parameters.yaml` file in `example`. It basically contains all intrinsic parameters and lever arms.
Note that ground truth file is optional and is used only for evaluation purposes.

Once images and scans are loaded, `createSfmJsonFile.py` allows you to automatically generate a JSON file readable by openMVG
with all the intrinsic parameters, filenames, and potentially poses parsed.

