
ORB_SLAM3 windows下编译说明

1. 编译Thirdparty/DBoW2
可能需要用到boost库
官网下载： https://www.boost.org/users/download/
https://sourceforge.net/projects/boost/files/boost-binaries/

我下载的是 boost_1_74_0-msvc-14.2-64.exe，双击解压得到boost_1_74_0目录，加入到DBoW2附加包含目录即可。

2. 编译 Thirdparty/g2o
g2o目录下的有可能会编译出错，我加了个版本 g2o_win。
可以直接打开 g2o_win\buildvs15 下的工程编译。
工程右键属性，C/C++，所有选项，加上附加选项 /bigobj ，与已有的项用空格隔开。
这个编的时间有点长，我把多处理器编译设为否，防止死机。

3. 解压字典 
ocabulary\ORBvoc.txt.tar.gz
解压到当前目录。

4. 编译ORM_SLAM3
我把cmakelist.txt 里opencv，eigen等的依赖去掉了，生成vs工程之后，要手动加到附加包含目录。
前面boost的目录也要加上，pangolin的目录也加上。
需要用到glew，官网下载，Thirdparty目录下已包含。
多处理器编译慎开，占用内存会非常大。


