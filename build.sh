echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=c++11 -fno-sized-deallocation -Wno-deprecated-declarations"

make -j8

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=c++11 -fno-sized-deallocation -Wno-deprecated-declarations"

make -j8

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=c++11 -fno-sized-deallocation -Wno-deprecated-declarations" 

make -j8
