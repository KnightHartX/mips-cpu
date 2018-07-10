# -*- coding: UTF-8 -*-
numbers = input()
if len(numbers) != 11:
    print("请输入一个11位二进制数！")
else:
    pass
print("输入的汉明码原始码为：" + numbers)
c1 = bool(int(numbers[0])) ^ bool(int(numbers[1])) ^ bool(int(
    numbers[3])) ^ bool(int(numbers[4])) ^ bool(int(numbers[6])) ^ bool(
        int(numbers[8])) ^ bool(int(numbers[10]))
print("c1的值为：", int(c1))
c2 = bool(int(numbers[0])) ^ bool(int(numbers[2])) ^ bool(int(
    numbers[3])) ^ bool(int(numbers[5])) ^ bool(int(numbers[6])) ^ bool(
        int(numbers[9])) ^ bool(int(numbers[10]))
print("c2的值为：", int(c2))
c3 = bool(int(numbers[1])) ^ bool(int(numbers[2])) ^ bool(int(
    numbers[3])) ^ bool(int(numbers[7])) ^ bool(int(numbers[8])) ^ bool(
        int(numbers[9])) ^ bool(int(numbers[10]))
print("c3的值为：", int(c3))
c4 = bool(int(numbers[4])) ^ bool(int(numbers[5])) ^ bool(int(
    numbers[6])) ^ bool(int(numbers[7])) ^ bool(int(numbers[8])) ^ bool(
        int(numbers[9])) ^ bool(int(numbers[10]))
print("c4的值为：", int(c4))
print("输出的汉明编码为：" + str(int(c1)) + str(int(c2)) + numbers[0] + str(int(c3)) +
      numbers[1] + numbers[2] + numbers[3] + str(int(c4)) + numbers[4] +
      numbers[5] + numbers[6] + numbers[7] + numbers[8] + numbers[9] +
      numbers[10])
