
ORB_SLAM3 windows下编译说明

建议vs2019编译，vs2017估计也可以。vs2015编译很慢且出错。

1. 下载boost库
官网下载： https://www.boost.org/users/download/
https://sourceforge.net/projects/boost/files/boost-binaries/
我下载的是 boost_1_74_0-msvc-14.2-64.exe，双击解压得到boost_1_74_0目录。

2. 解压字典 
ocabulary\ORBvoc.txt.tar.gz
解压到当前目录。

3. buildvs19 下打开ORM_SLAM3.sln，直接编译即可运行。
我试了mono_kitti，其他测试程序也都可以，只需要把包含目录、库目录、依赖库设置与mono_kitti相同即可。
mono_kitti 运行的命令行示例：
..\..\..\Vocabulary\ORBvoc.txt ..\..\..\Examples\Monocular\KITTI00-02.yaml F:\data\kitti\dataset\sequences\00

（下面这些库我已经编译好，可直接使用，你不需要从头编译）
从零编译：
1. 编译Thirdparty/DBoW2
用cmake创建工程，打开工程，boost目录加入到DBoW2附加包含目录，编译即可。

2. 编译 Thirdparty/g2o
用cmake创建工程，编译。

3. 编译ORM_SLAM3
我把cmakelist.txt 里opencv，eigen等的依赖去掉了，生成vs工程之后，要手动加到附加包含目录。
前面boost的目录也要加上，pangolin的目录也加上。
需要用到glew，官网下载，Thirdparty目录下已包含。
多处理器编译慎开，占用内存会非常大。


