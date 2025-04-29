CARLA 参与者（Actor）模块文档

概述

CARLA 是由巴塞罗那自治大学（UAB）计算机视觉中心（CVC）开发的开源自动驾驶模拟器，广泛应用于自动驾驶算法的测试与开发。参与者（Actor）模块是 CARLA 的核心组件，负责在模拟环境中创建、管理和销毁各种参与者，包括车辆、行人、交通信号灯、交通标志和传感器等。




核心功能

参与者模块的主要功能包括：





生成与注册：动态生成参与者并注册到系统中，分配唯一 ID。



状态管理：支持活跃和休眠状态切换，优化资源使用。



类型特定行为：为车辆、行人、交通信号灯等提供定制化控制。



ROS2 集成：通过 ROS2 接口实现外部控制，增强交互性。







详细功能解析

以下按文件逐一分析参与者模块的实现，包含更多关键代码和扩展的公式说明。

1. ActorData.cpp

功能

ActorData.cpp 定义了参与者数据结构，用于存储和恢复状态，支持休眠和唤醒。主要类包括：





FActorData：存储通用属性（如位置、旋转、速度、角速度）。



FVehicleData：车辆特定数据（如物理控制、灯光状态）。



FWalkerData：行人数据（如控制、存活状态）。



FTrafficSignData：交通标志数据（如模型、ID）。



FTrafficLightData：交通信号灯数据（如状态、控制器）。



FActorSensorData：传感器数据（如数据流）。

关键代码

FActorData::RecordActorData：记录参与者状态。

void FActorData::RecordActorData(FCarlaActor* CarlaActor, UCarlaEpisode* CarlaEpisode)
{
  AActor* Actor = CarlaActor->GetActor();
  FTransform Transform = Actor->GetTransform();
  Location = FDVector(Transform.GetLocation()) + CarlaEpisode->GetCurrentMapOrigin();
  Rotation = Transform.GetRotation();
  Scale = Transform.GetScale3D();
  UPrimitiveComponent* Component = Cast<UPrimitiveComponent>(Actor->GetRootComponent());
  if (Component)
  {
    bSimulatePhysics = Component->IsSimulatingPhysics();
    AngularVelocity = Component->GetPhysicsAngularVelocityInDegrees();
  }
  Velocity = Actor->GetVelocity();
}

FActorData::RespawnActor：重新生成参与者。

AActor* FActorData::RespawnActor(UCarlaEpisode* CarlaEpisode, const FActorInfo& Info)
{
  FTransform SpawnTransform = GetLocalTransform(CarlaEpisode);
  SpawnTransform.AddToTranslation(FVector(0,0,15));
  return CarlaEpisode->ReSpawnActorWithInfo(SpawnTransform, Info.Description);
}

公式说明





位置记录： 参与者的位置记录需要考虑地图原点偏移，以确保在全局坐标系中的正确性：

[ \text{Location} = \text{Transform.GetLocation()} + \text{CarlaEpisode.GetCurrentMapOrigin()} ]

其中：





(\text{Transform.GetLocation()})：参与者在局部坐标系中的位置向量（单位：厘米）。



(\text{CarlaEpisode.GetCurrentMapOrigin()})：当前地图的原点偏移向量（单位：厘米）。



(\text{Location})：全局坐标系中的位置向量，存储为 FDVector。



生成位置调整： 在重新生成参与者时，RespawnActor 将位置沿 Z 轴上移 15 厘米，以避免地面碰撞：

[ \text{SpawnTransform.Location.Z} = \text{GetLocalTransform(CarlaEpisode).Location.Z} + 15 ]

其中：





(\text{GetLocalTransform(CarlaEpisode).Location.Z})：局部坐标系中的 Z 坐标。



(15)：固定偏移量（单位：厘米）。



速度和角速度： 速度和角速度直接从 Unreal Engine 的物理组件获取：

[ \text{Velocity} = \text{Actor.GetVelocity()} ] [ \text{AngularVelocity} = \text{Component.GetPhysicsAngularVelocityInDegrees()} ]

其中：





(\text{Velocity})：线性速度向量（单位：厘米/秒）。



(\text{AngularVelocity})：角速度向量（单位：度/秒）。



2. ActorDispatcher.cpp

功能

UActorDispatcher 是参与者调度器，负责生成、注册、销毁参与者，并管理休眠/唤醒状态。

关键代码

UActorDispatcher::SpawnActor：生成并注册参与者。

TPair<EActorSpawnResultStatus, FCarlaActor*> UActorDispatcher::SpawnActor(
    const FTransform &Transform,
    FActorDescription Description,
    FCarlaActor::IdType DesiredId)
{
  if ((Description.UId == 0u) || (Description.UId > static_cast<uint32>(SpawnFunctions.Num())))
  {
    UE_LOG(LogCarla, Error, TEXT("Invalid ActorDescription '%s' (UId=%d)"), *Description.Id, Description.UId);
    return MakeTuple(EActorSpawnResultStatus::InvalidDescription, nullptr);
  }
  Description.Class = Classes[Description.UId - 1];
  FActorSpawnResult Result = SpawnFunctions[Description.UId - 1](Transform, Description);
  FCarlaActor* View = Result.IsValid() ? RegisterActor(*Result.Actor, std::move(Description), DesiredId) : nullptr;
  if (View)
  {
    ATagger::TagActor(*View->GetActor(), true);
  }
  return MakeTuple(Result.Status, View);
}

UActorDispatcher::RegisterActor：注册参与者并设置 ROS2 映射。

FCarlaActor* UActorDispatcher::RegisterActor(
    AActor &Actor, FActorDescription Description,
    FActorRegistry::IdType DesiredId)
{
  FCarlaActor* View = Registry.Register(Actor, Description, DesiredId);
  if (View)
  {
    Actor.OnDestroyed.AddDynamic(this, &UActorDispatcher::OnActorDestroyed);
    #if defined(WITH_ROS2)
    auto ROS2 = carla::ros2::ROS2::GetInstance();
    if (ROS2->IsEnabled())
    {
      std::string RosName;
      for (auto &&Attr : Description.Variations)
      {
        if (Attr.Key == "ros_name")
        {
          RosName = std::string(TCHAR_TO_UTF8(*Attr.Value.Value));
        }
      }
      const std::string id = std::string(TCHAR_TO_UTF8(*Description.Id));
      if (RosName == id && RosName.find("vehicle") != std::string::npos)
      {
        std::string VehicleName = "vehicle" + std::to_string(View->GetActorId());
        ROS2->AddActorRosName(static_cast<void*>(&Actor), VehicleName);
      }
    }
    #endif
  }
  return View;
}

公式说明





ID 验证： 参与者描述的唯一 ID（UId）必须满足：

[ 0 < \text{Description.UId} \leq \text{SpawnFunctions.Num()} ]

其中：





(\text{Description.UId})：参与者定义的索引。



(\text{SpawnFunctions.Num()})：已绑定生成函数的数量。



生成变换： 生成参与者时，使用提供的变换矩阵：

[ \text{Transform} = \begin{bmatrix} R & T \ 0 & 1 \end{bmatrix} ]

其中：





(R)：3x3 旋转矩阵，表示参与者的朝向。



(T)：平移向量，表示参与者的位置。



3. ActorRegistry.cpp

功能

FActorRegistry 维护参与者注册表，管理 ID 分配和状态。

关键代码

FActorRegistry::Register：注册参与者。

FCarlaActor* FActorRegistry::Register(AActor &Actor, FActorDescription Description, IdType DesiredId)
{
  IdType Id = ++FActorRegistry::ID_COUNTER;
  if (DesiredId != 0 && Id != DesiredId)
  {
    if (!Actors.Contains(DesiredId))
    {
      Id = DesiredId;
      if (ID_COUNTER < Id)
        ID_COUNTER = Id;
    }
  }
  Actors.Emplace(Id, &Actor);
  Ids.Emplace(&Actor, Id);
  TSharedPtr<FCarlaActor> View = MakeCarlaActor(Id, Actor, std::move(Description), crp::ActorState::Active);
  TSharedPtr<FCarlaActor>& Result = ActorDatabase.Emplace(Id, MoveTemp(View));
  check(static_cast<size_t>(Actors.Num()) == ActorDatabase.Num());
  return Result.Get();
}

FActorRegistry::MakeCarlaActor：创建参与者实例。

TSharedPtr<FCarlaActor> FActorRegistry::MakeCarlaActor(
    IdType Id,
    AActor &Actor,
    FActorDescription Description,
    crp::ActorState InState)
{
  auto Info = MakeShared<FActorInfo>();
  Info->Description = std::move(Description);
  ATagger::GetTagsOfTaggedActor(Actor, Info->SemanticTags);
  Info->BoundingBox = UBoundingBoxCalculator::GetActorBoundingBox(&Actor);
  if (Info->Description.Id.IsEmpty())
  {
    Info->Description.Id = TEXT("static.") + CarlaGetRelevantTagAsString(Info->SemanticTags);
  }
  Info->SerializedData.id = Id;
  Info->SerializedData.bounding_box = Info->BoundingBox;
  auto Type = FActorRegistry_GetActorType(&Actor);
  TSharedPtr<FCarlaActor> CarlaActor = FCarlaActor::ConstructCarlaActor(
      Id, &Actor, std::move(Info), Type, InState, Actor.GetWorld());
  return CarlaActor;
}

公式说明





ID 分配： 新参与者的 ID 通过全局计数器递增：

[ \text{Id} = \text{FActorRegistry::ID_COUNTER} + 1 ]

如果指定了期望 ID（DesiredId）且未被占用：

[ \text{Id} = \text{DesiredId}, \quad \text{if } \text{Actors.Contains(DesiredId)} = \text{false} ]

同时更新计数器：

[ \text{FActorRegistry::ID_COUNTER} = \max(\text{FActorRegistry::ID_COUNTER}, \text{Id}) ]



边界框计算： 参与者的边界框通过以下公式计算：

[ \text{BoundingBox} = \text{UBoundingBoxCalculator::GetActorBoundingBox}(\text{Actor}) ]

边界框通常表示为：

[ \text{BoundingBox} = { \text{Center}, \text{Extent} } ]

其中：





(\text{Center})：边界框中心位置向量。



(\text{Extent})：边界框在各轴上的半尺寸向量。



4. ActorROS2Handler.cpp

功能

ActorROS2Handler 将 ROS2 控制命令转换为 CARLA 车辆控制。

关键代码

ActorROS2Handler::operator()：转换控制命令。

void ActorROS2Handler::operator()(carla::ros2::VehicleControl &Source)
{
  if (!_Actor) return;
  ACarlaWheeledVehicle *Vehicle = Cast<ACarlaWheeledVehicle>(_Actor);
  if (!Vehicle) return;
  FVehicleControl NewControl;
  NewControl.Throttle = Source.throttle;
  NewControl.Steer = Source.steer;
  NewControl.Brake = Source.brake;
  NewControl.bHandBrake = Source.hand_brake;
  NewControl.bReverse = Source.reverse;
  NewControl.bManualGearShift = Source.manual_gear_shift;
  NewControl.Gear = Source.gear;
  Vehicle->ApplyVehicleControl(NewControl, EVehicleInputPriority::User);
}

公式说明





控制映射： ROS2 控制命令直接映射到 CARLA 控制参数：

[ \text{NewControl.Throttle} = \text{Source.throttle}, \quad \text{range: } [0, 1] ] [ \text{NewControl.Steer} = \text{Source.steer}, \quad \text{range: } [-1, 1] ] [ \text{NewControl.Brake} = \text{Source.brake}, \quad \text{range: } [0, 1] ]

其中：





(\text{throttle})：油门强度。



(\text{steer})：转向角度（归一化）。



(\text{brake})：刹车强度。



优先级设置： 控制应用时使用用户优先级：

[ \text{Priority} = \text{EVehicleInputPriority::User} ]

这确保用户输入优先于其他控制源（如自动驾驶）。



5. ActorSpawnResult.cpp

功能

FActorSpawnResult 定义生成结果状态的描述。

关键代码

FActorSpawnResult::StatusToString：状态转字符串。

FString FActorSpawnResult::StatusToString(EActorSpawnResultStatus InStatus)
{
  switch (InStatus)
  {
    case EActorSpawnResultStatus::Success:
      return TEXT("Success");
    case EActorSpawnResultStatus::InvalidDescription:
      return TEXT("Spawn failed because of invalid actor description");
    case EActorSpawnResultStatus::Collision:
      return TEXT("Spawn failed because of collision at spawn position");
    case EActorSpawnResultStatus::UnknownError:
    default:
      return TEXT("Unknown error while trying to spawn actor");
  }
}

公式说明





状态枚举： 生成状态通过枚举定义：

[ \text{EActorSpawnResultStatus} \in { \text{Success}, \text{InvalidDescription}, \text{Collision}, \text{UnknownError} } ]

状态数量受静态断言约束：

[ \text{sizeof}(\text{EActorSpawnResultStatus}) = 4 ]



6. CarlaActor.cpp

功能

CarlaActor.cpp 定义 FCarlaActor 及其派生类，管理不同类型参与者的行为。

关键类





FCarlaActor：基类，管理 ID、状态、变换。



FVehicleActor：车辆，处理物理、灯光、自动驾驶。



FSensorActor：传感器，管理数据流。



FTrafficSignActor：交通标志，存储标志信息。



FTrafficLightActor：交通信号灯，控制状态。



FWalkerActor：行人，处理移动和动画。



FOtherActor：其他类型，通用处理。

关键代码

FCarlaActor::SetActorGlobalTransform：设置全局变换。

void FCarlaActor::SetActorGlobalTransform(
    const FTransform& Transform, ETeleportType TeleportType)
{
  if (IsDormant())
  {
    ActorData->Location = FDVector(Transform.GetLocation());
    ActorData->Rotation = Transform.GetRotation();
    ActorData->Scale = Transform.GetScale3D();
  }
  else
  {
    FTransform LocalTransform = Transform;
    ALargeMapManager* LargeMap = UCarlaStatics::GetLargeMapManager(World);
    if (LargeMap)
    {
      LocalTransform = LargeMap->LocalToGlobalTransform(LocalTransform);
    }
    GetActor()->SetActorRelativeTransform(LocalTransform, false, nullptr, TeleportType);
  }
}

FWalkerActor::SetWalkerState：设置行人状态。

ECarlaServerResponse FWalkerActor::SetWalkerState(
    const FTransform& Transform,
    carla::rpc::WalkerControl WalkerControl)
{
  FVector NewLocation = Transform.GetLocation();
  UCapsuleComponent* Capsule = Cast<UCapsuleComponent>(GetActor()->GetRootComponent());
  if (Capsule)
  {
    NewLocation.Z += Capsule->GetScaledCapsuleHalfHeight();
  }
  FTransform NewTransform = Transform;
  NewTransform.SetLocation(NewLocation);
  if (IsDormant())
  {
    FWalkerData* WalkerData = GetActorData<FWalkerData>();
    WalkerData->WalkerControl = WalkerControl;
  }
  else
  {
    auto Controller = Cast<AWalkerController>(Cast<APawn>(GetActor())->GetController());
    if (Controller)
    {
      Controller->ApplyWalkerControl(WalkerControl);
    }
  }
  SetActorGlobalTransform(NewTransform);
  return ECarlaServerResponse::Success;
}

公式说明





行人位置调整： 行人位置沿 Z 轴上移半个胶囊体高度，以适应导航网格：

[ \text{NewLocation.Z} = \text{Transform.GetLocation().Z} + \text{Capsule.GetScaledCapsuleHalfHeight()} ]

其中：





(\text{Transform.GetLocation().Z})：输入变换的 Z 坐标。



(\text{Capsule.GetScaledCapsuleHalfHeight()})：胶囊体高度的一半（单位：厘米）。



全局变换设置： 在活跃状态下，全局变换可能需要转换为局部变换：

[ \text{LocalTransform} = \text{LargeMap.LocalToGlobalTransform}(\text{Transform}) ]

其中：





(\text{Transform})：全局变换矩阵。



(\text{LocalTransform})：局部变换矩阵，考虑大地图的坐标转换。



休眠状态变换： 在休眠状态下，直接更新数据：

[ \text{ActorData.Location} = \text{FDVector}(\text{Transform.GetLocation()}) ] [ \text{ActorData.Rotation} = \text{Transform.GetRotation()} ] [ \text{ActorData.Scale} = \text{Transform.GetScale3D()} ]



应用场景





自动驾驶测试：模拟车辆和行人，验证自动驾驶算法。



传感器模拟：管理传感器数据流，测试感知系统。



交通管理：控制交通信号灯，模拟交通规则。



ROS2 集成：通过 ROS2 实现外部控制。



注意事项





休眠限制：休眠状态下部分功能（如力应用）不可用。



ROS2 配置：需正确设置 ROS2 环境（参考 ROS2 文档).



物理差异：车辆和行人物理模拟方式不同，需类型检查。



大地图支持：使用 ALargeMapManager 处理全局/局部坐标。



结论

CARLA 参与者模块通过模块化设计和灵活状态管理，高效实现了参与者生命周期管理。UActorDispatcher 和 FActorRegistry 支持动态生成和注册，FCarlaActor 及其派生类提供类型特定行为，ActorROS2Handler 增强外部集成能力，为自动驾驶模拟提供了强大支持。
