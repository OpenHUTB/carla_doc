import glob
import os
import sys
import carla
import argparse
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import time
import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler

from keras.models import Sequential
from keras.layers import LSTM
from keras.layers import Dense, Activation, Dropout
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


def load_data(filename, time_step):
    '''
    filename:
    instruction: file address, note '/'

    time_step: int
    instruction: how many previous samples are used to predict the next sample, it is the same with the time_steps of that in LSTM
    '''
    df = pd.read_csv(filename)
    data = df.value
    plt.title('original data')
    plt.plot(data)
    # plt.savefig('original data.png')
    plt.show()
    # using a list variable to rebuild a dataset to store previous time_step samples and another predicted sample
    result = []
    for index in range(len(data) - time_step):
        result.append(data[index:index + time_step + 1])

    # variable 'result' can be (len(data)-time_step) * (time_step + 1), the last column is predicted sample.
    return np.array(result)

def build_lstm_model(layer):
    '''
    layer: list
    instruction: the number of neurons in each layer
    '''
    model = Sequential()
    # set the first hidden layer and set the input dimension
    model.add(LSTM(
        input_shape=(1, layer[0]), units=layer[1], return_sequences=True
    ))
    model.add(Dropout(0.2))

    # add the second layer
    model.add(LSTM(
        units=layer[2], return_sequences=False
    ))
    model.add(Dropout(0.2))

    # add the output layer with a Dense
    model.add(Dense(units=layer[3], activation='linear'))
    model.compile(loss='mse', optimizer='adam')

    return model

def plot_curve(true_data, predicted_data):
    '''
    true_data: float32
    instruction: the true test data
    predicted_data: float32
    instruction: the predicted data from the model
    '''
    fig, ax = plt.subplots()
    ax.plot(true_data, label='True data')
    ax.plot(predicted_data, label='Predicted data')
    ax.legend()

    # 设置y轴的格式化器为整数
    ax.yaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False, useMathText=True))
    ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))

    plt.savefig('result.png')
    plt.show()

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--traffic_id',
        metavar='I',
        default=1,
        type=int,
        help='traffic light id')
    argparser.add_argument(
        '--color_id',
        metavar='C',
        default=1,
        type=int,
        help='traffic light color id')
    argparser.add_argument(
        '--color_time',
        metavar='T',
        default=20,
        type=int,
        help='set traffic light time')
    args = argparser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)  # 设置超时
    world = client.get_world()  # 获取世界对象

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  #仿真环境会以0.05秒的时间前进。
    world.apply_settings(settings)

    # 获取所有正在行驶的车辆列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    #print(vehicle_list)
    # 统计路口
    xmax = -11
    xmin = -77
    ymax = 57
    ymin = -13

    # 创建一个空的字典
    vehicle_positions = {}

    # 创建一个空的列表，用于存储车流量数据
    traffic_data = []
    # 创建两个空列表，用于存储时刻和车流量数据
    timestamps = []
    traffic_flow = []

    # 循环推进仿真时间并统计车流量
    for t in range(1000):  # 假设仿真100个时间步
        # 推进仿真时间
        world.tick()

        # 重置车流量计数
        count = 0

        # 遍历所有正在行驶的车辆
        for vehicle in vehicle_list:
            # 获取车辆的位置信息
            location = vehicle.get_location()
            x = location.x
            y = location.y
            z = location.z
            if x <= xmax and x >= xmin and y <= ymax and y >= ymin:
                count += 1
            vehicle_positions[vehicle.id] = (x, y, z)
    
        # 将时间和车流量数据添加到traffic_data列表中
        traffic_data.append((t, count))
        # 将时间和车流量数据添加到相应的列表中
        timestamps.append(t)
        traffic_flow.append(count)
   
 # 将车流量数据写入文件
    filename = "traffic_data.csv"
    with open(filename, "w", newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(["head", "value"])  # 写入表头
        for data in traffic_data:
            writer.writerow([data[0], data[1]])  # 写入数据

    data = load_data('traffic_data.csv', 200)

    # normalize the data and split it into train and test set
    scaler = MinMaxScaler(feature_range=(0, 1))
    dataset = scaler.fit_transform(data)
    # define a variable to represent the ratio of train/total and split the dataset
    train_count = int(0.7 * len(dataset))
    x_train_set, x_test_set = dataset[:train_count, :-1], dataset[train_count:, :-1]
    y_train_set, y_test_set = dataset[:train_count, -1], dataset[train_count:, -1]

    # reshape the data to satisfy the input acquirement of LSTM
    x_train_set = x_train_set.reshape(x_train_set.shape[0], 1, x_train_set.shape[1])
    x_test_set = x_test_set.reshape(x_test_set.shape[0], 1, x_test_set.shape[1])
    y_train_set = y_train_set.reshape(y_train_set.shape[0], 1)
    y_test_set = y_test_set.reshape(y_test_set.shape[0], 1)
    layer = [200, 500, 500, 1]
    model = build_lstm_model(layer)

    # train the model and use the validation part to validate
    model.fit(x_train_set, y_train_set, batch_size=100, epochs=25, validation_split=0.2)

    # do the prediction
    y_predicted = model.predict(x_test_set)
    # plot the predicted curve and the original curve
    # fill some zeros to get a (len, 51) array
    temp = np.zeros((len(y_test_set), 200))
    origin_temp = np.hstack((temp, y_test_set))
    predict_temp = np.hstack((temp, y_predicted))

    # tranform the data back to the original one
    origin_test = scaler.inverse_transform(origin_temp)
    predict_test = scaler.inverse_transform(predict_temp)
    predict_test = np.round(predict_test).astype(int)
    plot_curve(origin_test[:, -1], predict_test[:, -1])



if __name__ == '__main__':
    main()
