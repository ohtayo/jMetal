import numpy as np

# 目的数
obj = 4
# 1目的の分割数
sep = 5

# 重み計算ループ
temp = np.zeros([1, obj])  # weightの初期化
weights = np.array([[]])  # 作成したweightの格納先
while True:
    # print(temp) # for debug

    # 合計がsep-1の場合のみ，weightsに格納する
    if (np.sum(temp) == (sep - 1)):
        # 最初の一回は直接格納
        if (weights.size == 0):
            # print(temp) # for debug
            weights = temp
        else:
            # print(temp) # for debug
            weights = np.concatenate([weights, temp])

    # 1カウント
    temp[0, -1] += 1
    # 桁上がり処理
    for o in reversed(range(1, obj)):
        # print(o)
        if not (temp[0, o] < sep):
            temp[0, o] = 0
            temp[0, o - 1] += 1
    # 最上位桁までカウントしたら終了
    if not (temp[0, 0] < sep):
        break

# weightsを0-1スケールにする
weights /= (sep - 1)

# weightsの情報表示
print('number of weight = ' + str(len(weights)))
print('weights = ')
print(weights)

# weightsの保存
file = 'W' + str(obj) + 'D_' + str(len(weights)) + '.dat'
print('weights are saved to ' + file)
np.savetxt(file, weights, fmt='%.8f', delimiter=' ')
