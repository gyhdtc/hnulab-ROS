#include <fstream>
int main() {
  std::ofstream outFile; //打开文件
  outFile.open("/home/ipc/diansai_ws/src/real_ur5_test/Test.txt",std::ios::app | std::ios::out);
  int arr[5] = {1, 2, 3, 4, 5};
  for (int i = 0; i < 5; i++) {
    //写入数据
    outFile << arr[i];
  }
  //关闭文件
  outFile << "\n";
  outFile.close();

return 0;

}
