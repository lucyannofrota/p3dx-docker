#!/bin/bash
echo "---<ultralitics_yolo_v5_install.bash>---"
set -e

cd $WORKSPACE

rm -r -f build install log | sed 's/^/  /'
mkdir -p $WORKSPACE/src/smap/smap_perception/detectors/smap_yolo_v5
cd $WORKSPACE/src/smap/smap_perception/detectors/smap_yolo_v5



vcs import < smap_yolo_v5.repos
cd yolov5

# File
find  . -maxdepth 1 -type f ! \( -ipath '*.md' -o -ipath '*.git*' -o -ipath '*grep*' -o -ipath '*ignore' -o -name '.' -o -name '.pre-commit-config.yaml' -o -name '__init__.py' \) > .gitignore_f
while read file; do (echo "${file}" | cut -c 3-) >> .gitignore_f_; done < .gitignore_f
while read file; do (cp $file ../$file); done < .gitignore_f_

# Dir
find  . -maxdepth 1 -type d ! \( -ipath '*.md' -o -ipath '*.git*' -o -ipath '*grep*' -o -ipath '*ignore' -o -name '.' \) > .gitignore_d
#while read folder; do (cut -c 3-) >> .gitignore_d_; done < .gitignore_d
cut -c 3- .gitignore_d > .gitignore_d_
while read folder; do (cp -r --parents $folder ./..); done < .gitignore_d_
while read folder; do (echo "${folder}/*") >> .gitignore_d__; done < .gitignore_d_

# Merging .gitignore
printf "\n" >> .gitignore
echo "## ultralytics/yolov5" >> .gitignore
printf "\n" >> .gitignore
echo "# Files" >> .gitignore
while read in; do ( echo ${in}\ ) >> .gitignore; done < .gitignore_f_
printf "\n" >> .gitignore
echo "# Folders" >> .gitignore
while read in; do ( echo ${in}\ ) >> .gitignore; done < .gitignore_d__

# Should be executed just once
#while read in; do (grep -qaexF $in ../.gitignore || echo ${in}\ ) >> ../.gitignore; done < .gitignore

cd ../

rm -fr yolov5

pip install --upgrade --upgrade-strategy only-if-needed -r $WORKSPACE/src/smap/smap_perception/detectors/smap_yolo_v5/jetson-requirements.txt

cd $WORKSPACE/src/smap/smap_perception/detectors/smap_yolo_v5

rm -fr weights
mkdir -p weights
cd weights
python3 ../export.py --weights yolov5s.pt --include engine --device 0 --inplace --imgsz 640 --data ../data/coco128.yaml --opset 16

echo "---</ultralitics_yolo_v5_install.bash>---"