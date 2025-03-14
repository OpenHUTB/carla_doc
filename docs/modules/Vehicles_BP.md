# 载具蓝图

## 新两轮基座 [BP_Base2wheeledNew](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/2Wheeled/BP_Base2wheeledNew.uasset)

## 车辆棋子基座 [BaseVehiclePawn](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/BaseVehiclePawn.uasset)
其中`事件图表`中的`中断VehicleControl`模块接受`Vehicle/VehicleControl.h`中所定义的车辆控制变量：
```cpp
UPROPERTY(Category = "Vehicle Control", EditAnywhere, BlueprintReadWrite)
float Brake = 0.0f;
```

## [N轮车辆棋子基座](../tuto_A_add_vehicle.md#add-a-n-wheeled-vehicle) [BaseVehiclePawnNW](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/BaseVehiclePawnNW.uasset)
## 车辆工厂 [VehicleFactory](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/VehicleFactory.uasset)


## 两轮载具 [2Wheeled](https://bitbucket.org/carla-simulator/carla-content/src/master/Blueprints/Vehicles/2Wheeled/)

