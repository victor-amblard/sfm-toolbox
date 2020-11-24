cwd=$(pwd)
dir='/dest/directory'
zipFile='/path/to/zip/file.zip' 
unzip "$zipfile" -d "$dir"
cd $dir
mv file/* ./
rm -rf file/
mkdir imgs
mv *.jpg imgs
mkdir scans
touch parameters.yaml
cd $cwd
python3 undistortImg.py $dir/imgs 
cd $dir/imgs
mkdir raw
mkdir rectified
mv *_rectified.jpg rectified
mv *.jpg raw

