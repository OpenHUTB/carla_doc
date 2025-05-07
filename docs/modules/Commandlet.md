# CarlaUnreal Commandlet 模块详细说明文档

## 1. 概述

CarlaUnreal 是为 CARLA 模拟器设计的虚幻引擎（Unreal Engine）插件，用于支持自动驾驶模拟环境的地图生成、资产管理和场景渲染。Commandlet 模块是该插件的核心组件之一，提供一系列命令行工具（Commandlets），用于自动化处理地图资产的加载、整理和打包。这些工具主要在编辑器环境下运行，适用于大规模地图的材质应用、语义分割和资产“烹饪”（打包为最终格式）。

本文档详细说明了 Commandlet 模块中的三个关键文件：

- **LoadAssetMaterialsCommandlet.cpp**：负责加载和应用道路材质（贴花和 Carla 材质）到地图。
- **MoveAssetsCommandlet.cpp**：根据语义分割规则将资产移动到指定文件夹。
- **PrepareAssetsForCookingCommandlet.cpp**：准备地图和道具资产以进行打包（烹饪）。

文档结构如下：
- 每个 Commandlet 的功能、输入参数、执行流程、关键依赖和注意事项。
- 使用场景和示例命令。
- 代码结构分析。
- 错误处理和调试建议。

---

## 2. Commandlet 详细说明

### 2.1 LoadAssetMaterialsCommandlet

#### 功能
`LoadAssetMaterialsCommandlet` 是一个命令行工具，用于为 CARLA 地图加载和应用道路材质，包括贴花（Decals，如泥土、裂缝、轮胎痕迹、井盖等）和 Carla 特定的材质。它支持单张地图和分块地图（Tiled Maps），通过 OpenDrive 数据和配置文件生成逼真的道路细节。

#### 输入参数
- **PackageName**：资产包名称，用于定位地图和配置文件。
- **Params**：命令行参数字符串，格式为 `PackageName=<名称>`。

#### 执行流程
1. **初始化**：
   - 构造函数中初始化贴花名称映射表（`DecalNamesMap`），包含多种贴花类型（如 `dirt1`、`crack1`、`manhole1`）及其材质路径。
   - 加载道路绘制器蓝图（`RoadPainterBlueprint`），路径为 `/Game/Carla/Blueprints/LevelDesign/RoadPainterPreset`。
   - 设置命令let标志：`IsEditor=true`、`IsClient=false`、`IsServer=false`、`LogToConsole=true`。

2. **参数解析**：
   - `ParseParams` 解析命令行参数，提取 `PackageName`。
   - `GetAssetsPathFromPackage` 读取包配置文件（`PackageName.Package.json`），获取地图路径（`MapsPaths`）和道具路径（`PropsPaths`）。

3. **加载地图资产**：
   - `LoadAssetsMaterials` 使用 `UObjectLibrary` 加载地图资产（`UWorld` 类型）。
   - 检查地图是否包含道路网格（名称包含 `Roads` 或 `Road` 的 `AStaticMeshActor`）。
   - 根据地图名称是否包含 `_Tile_` 判断是否为分块地图。

4. **应用材质**：
   - `ApplyRoadPainterMaterials` 根据地图类型执行：
     - **非分块地图**：直接生成贴花。
     - **分块地图**：
       - 读取 `TilesInfo.txt` 获取瓦片中心坐标（`FirstTileCenterX`、`FirstTileCenterY`）和大小（`Size`）。
       - 加载 OpenDrive 文件（`.xodr`），使用 `UOpenDrive::LoadXODR` 和 `OpenDriveParser` 解析道路数据。
       - 解析地图名称（例如 `MapName_Tile_200_400`）提取瓦片索引（`XIndex`、`YIndex`），计算瓦片位置。
       - 读取 `roadpainter_decals.json` 获取贴花配置（`DecalsProperties`），包括材质、生成数量、缩放范围和随机偏航角。
       - 在瓦片范围内随机生成贴花，确保贴花位于道路上（通过 `XODRMap->GetClosestWaypointOnRoad` 获取最近道路点）。
       - 设置贴花的缩放、旋转和位置，生成 `ADecalActor`。
   - 贴花生成考虑随机性（位置、缩放、偏航），以增强视觉真实感。

5. **保存修改**:column
   - 使用 `UEditorLoadingAndSavingUtils::SaveDirtyPackages` 保存修改后的地图包。

#### 关键依赖
- **roadpainter_decals.json**：定义贴花类型、数量、缩放范围（`decal_min_scale`、`decal_max_scale`）、随机偏航（`decal_random_yaw`）等。
- **TilesInfo.txt**：存储分块地图的瓦片信息，格式为 `X,Y,Size`（单位：米）。
- **OpenDrive 文件（.xodr）**：提供道路几何信息，用于贴花定位。
- **RoadPainterPreset 蓝图**：用于道路绘制逻辑。

#### 注意事项
- 仅在编辑器环境下运行（`WITH_EDITORONLY_DATA`）。
- 确保 `roadpainter_decals.json` 和 `TilesInfo.txt` 文件存在且格式正确，否则会导致警告日志（如 `Could not read TilesInfo.txt file`）。
- 分块地图名称需包含 `_Tile_X_Y` 格式，例如 `MapName_Tile_200_400`。
- 贴花生成依赖 OpenDrive 数据，`.xodr` 文件需与地图名称一致。
- 如果地图不包含道路网格，贴花不会生成。
- 保存操作可能覆盖现有包，建议备份。

#### 错误处理
- **文件缺失**：检查 `TilesInfo.txt` 和 `roadpainter_decals.json` 是否存在，路径是否正确。
- **OpenDrive 解析失败**：验证 `.xodr` 文件格式，确保与地图名称匹配。
- **贴花生成失败**：检查日志，确认瓦片范围和道路数据是否有效。

---

### 2.2 MoveAssetsCommandlet

#### 功能
`MoveAssetsCommandlet` 用于根据语义分割规则将静态网格资产（`UStaticMesh`）从地图文件夹移动到指定的语义分类文件夹（如 `ROAD`、`ROADLINE`、`TERRAIN` 等）。它通过检查资产名称中的语义标签（如 `R_ROAD1`、`R_MARKING1`）进行分类，适用于整理 CARLA 地图资产以便后续处理。

#### 输入参数
- **PackageName**：资产包名称。
- **Maps**：要处理的地图名称列表，多个地图以空格分隔。
- **Params**：命令行参数字符串，格式为 `PackageName=<名称> Maps=<地图1 地图2 ...>`。

#### 执行流程
1. **初始化**：
   - 构造函数设置命令let标志：`IsEditor=true`、`IsClient=false`、`IsServer=false`、`LogToConsole=true`。

2. **参数解析**：
   - `ParseParams` 解析命令行参数，提取 `PackageName` 和 `MapNames`（地图名称数组）。
   - 使用 `FParse::Value` 提取 `PackageName` 和 `Maps`，并将 `Maps` 按空格分割为数组。

3. **移动资产**：
   - `MoveAssets` 遍历每个地图名称，调用 `MoveAssetsFromMapForSemanticSegmentation`。
   - `MoveAssetsFromMapForSemanticSegmentation`：
     - 构建源路径（`/Game/PackageName/Maps/MapName`）。
     - 使用 `UObjectLibrary` 加载源路径下的静态网格资产，存储到 `MapContents`。
     - 遍历资产，检查名称中的语义标签（通过 `SSTags` 定义，如 `R_ROAD1`、`R_MARKING1`），分类到以下类别：
       - `ROAD`：`R_ROAD1` 或 `R_ROAD2`
       - `ROADLINE`：`R_MARKING1` 或 `R_MARKING2`
       - `TERRAIN`：`R_TERRAIN` 或未匹配的资产
       - `SIDEWALK`：`R_SIDEWALK1` 或 `R_SIDEWALK2`
       - `CURB`：`R_CURB1` 或 `R_CURB2`
       - `GUTTER`：`R_GUTTER1` 或 `R_GUTTER2`
     - 将分类后的资产存储到 `AssetDataMap`（`TMap<FString, TArray<UObject *>>`）。
     - 遍历 `AssetDataMap`，调用 `MoveFiles` 将资产移动到目标路径（`/Game/PackageName/Static/Category/MapName`）。
   - `MoveFiles` 使用 `AssetToolsModule::RenameAssets` 执行重命名操作，实现资产移动。

4. **清理**：
   - 每次处理完地图后，调用 `AssetsObjectLibrary->ClearLoaded` 释放内存。

#### 关键依赖
- **SSTags.h**：定义语义分割标签（如 `R_ROAD1`、`R_MARKING1`）。
- **AssetTools 模块**：提供资产重命名功能。

#### 注意事项
- 仅在编辑器环境下运行（`WITH_EDITORONLY_DATA`）。
- 资产名称需包含语义标签（如 `R_ROAD1`），否则默认归类到 `TERRAIN`。
- 目标路径必须有效，否则触发断言失败（`check(DestPath.Len() > 0)`）。
- 移动操作会修改资产路径，建议备份原始资产。
- 如果资产包名称或路径错误，可能导致加载失败。

#### 错误处理
- **资产未加载**：检查源路径（`/Game/PackageName/Maps/MapName`）是否包含静态网格。
- **分类错误**：验证资产名称是否遵循语义标签规则。
- **路径错误**：检查日志，确认 `PackageName` 和 `MapName` 是否正确。

---

### 2.3 PrepareAssetsForCookingCommandlet

#### 功能
`PrepareAssetsForCookingCommandlet` 用于准备地图和道具资产以进行“烹饪”（打包为最终格式）。它支持单张地图和分块地图，应用 Carla 材质，生成瓦片路径文件，并处理 OpenDrive 数据以添加生成点（Spawn Points）。此外，它可以生成道具地图（`PropsMap`）并保存路径文件供后续使用。

#### 输入参数
- **PackageName**：资产包名称。
- **OnlyPrepareMaps**：布尔值，指示是否仅准备地图（忽略道具）。
- **Params**：命令行参数字符串，格式为 `PackageName=<名称> OnlyPrepareMaps=<true/false>`。

#### 执行流程
1. **初始化**：
   - 构造函数加载 Carla 默认材质（如 `M_Road_03_Tiled_V2`、`MI_LargeLandscape_Grass` 等）。
   - 设置命令let标志：`IsEditor=true`、`IsClient=false`、`IsServer=false`、`LogToConsole=true`。

2. **参数解析**：
   - `ParseParams` 解析命令行参数，提取 `PackageName` 和 `bOnlyPrepareMaps`。
   - `GetAssetsPathFromPackage` 读取包配置文件（`PackageName.Package.json`），获取地图（`MapsPaths`）和道具（`PropsPaths`）路径。

3. **准备地图**：
   - `PrepareMapsForCooking` 遍历每个地图：
     - **非分块地图**：
       - 调用 `LoadWorld` 加载基础地图（`/Game/Carla/Maps/BaseMap`）。
       - 使用 `SpawnMeshesToWorld` 生成静态网格，应用 Carla 材质（如果 `bUseCarlaMapMaterials=true`）。
       - 调用 `SaveWorld` 保存地图到指定路径。
       - 调用 `DestroySpawnedActorsInWorld` 销毁临时生成的演员。
     - **分块地图**：
       - 调用 `IsMapInTiles` 检查资产名称是否包含 `_Tile_`。
       - 读取 `TilesInfo.txt` 获取瓦片信息（中心坐标和大小，单位：米，转换为厘米）。
       - 调用 `LoadWorldTile` 加载瓦片基础地图（`/Game/Carla/Maps/TestMaps`）。
       - 逐个生成瓦片（`_Tile_i_j`），调用 `SpawnMeshesToWorld` 和 `SaveWorld`。
       - 调用 `LoadLargeMapWorld` 加载大型地图基础（`/Game/Carla/Maps/BaseLargeMap`）。
       - 生成 `ALargeMapManager`，设置瓦片路径、名称、偏移和大小，调用 `GenerateMap` 保存主地图。
   - `SpawnMeshesToWorld`：
     - 加载静态网格资产，过滤包含 `light` 或 `sign` 的网格（通过 `ValidateStaticMesh`）。
     - 根据语义标签（如 `R_ROAD1`、`R_MARKING1`）应用对应材质。
     - 设置复杂碰撞为简单碰撞（`CTF_UseComplexAsSimple`）。
   - `SaveWorld`：
     - 如果存在 `.xodr` 文件且 `bGenerateSpawnPoints=true`，生成 `AOpenDriveActor`，添加路线和生成点。
     - 保存包到指定路径，检查是否已存在以避免覆盖。

4. **准备道具**：
   - 如果 `bOnlyPrepareMaps=false`，调用 `PreparePropsForCooking`：
     - 加载道具资产（`PropsPaths`），生成到基础地图（`PropsMap`）。
     - 保存道具地图到 `/Game/PackageName/Maps/PropsMap`。
     - 销毁临时演员。

5. **生成路径文件**：
   - `GenerateMapPathsFile` 生成 `MapPaths.txt`（Windows 格式，换行分隔）和 `MapPathsLinux.txt`（Linux 格式，加号分隔），记录地图和瓦片路径。
   - `GeneratePackagePathFile` 生成 `PackagePath.txt`，记录包配置文件路径。

6. **保存修改**：
   - 使用 `UEditorLoadingAndSavingUtils::SaveDirtyPackages` 保存所有修改。

#### 关键依赖
- **PackageName.Package.json**：定义地图和道具路径。
- **TilesInfo.txt**：存储分块地图的瓦片信息，格式为 `X,Y,Size`。
- **OpenDrive 文件（.xodr）**：用于生成路线和生成点。
- **SSTags.h**：定义语义分割标签。
- **Carla 材质**：如 `M_Road_03_Tiled_V2` 等。

#### 注意事项
- 仅在编辑器环境下运行（`WITH_EDITORONLY_DATA`）。
- 确保 `TilesInfo.txt` 和 `.xodr` 文件存在且格式正确。
- 分块地图的瓦片名称需遵循 `_Tile_i_j` 格式。
- 道具路径需正确配置，否则可能导致加载失败。
- 保存操作会检查目标路径是否已存在，避免覆盖，但仍建议备份。
- 过滤 `light` 或 `sign` 的网格可能导致部分资产被忽略。

#### 错误处理
- **瓦片信息缺失**：检查 `TilesInfo.txt` 是否存在，格式是否为 `X,Y,Size`。
- **地图加载失败**：验证基础地图路径（`BaseMap`、`TestMaps`、`BaseLargeMap`）是否正确。
- **保存失败**：检查目标路径是否可写，包是否已存在。
- **材质应用错误**：确认语义标签和材质路径匹配。

---

## 3. 使用场景和示例

### 3.1 加载和应用道路材质
**场景**：为地图添加逼真的道路贴花（如裂缝、泥土）。
**命令**：
```bash
UE4Editor.exe MyProject.uproject -run=LoadAssetMaterialsCommandlet PackageName=MyPackage
```
**预期结果**：地图加载指定贴花，保存到原包路径。

### 3.2 整理资产
**场景**：按语义分割规则整理地图资产。
**命令**：
```bash
UE4Editor.exe MyProject.uproject -run=MoveAssetsCommandlet PackageName=MyPackage Maps=Map1 Map2
```
**预期结果**：资产移动到 `/Game/MyPackage/Static/ROAD/Map1` 等路径。

### 3.3 准备资产进行烹饪
**场景**：打包地图和道具为最终格式。
**命令**：
```bash
UE4Editor.exe MyProject.uproject -run=PrepareAssetsForCookingCommandlet PackageName=MyPackage OnlyPrepareMaps=false
```
**预期结果**：生成地图、瓦片和道具地图，保存路径文件。

---

## 4. 代码结构分析

### 4.1 LoadAssetMaterialsCommandlet
- **核心类**：`ULoadAssetMaterialsCommandlet`
- **关键函数**：
  - `ApplyRoadPainterMaterials`：处理贴花生成，支持分块和非分块地图。
  - `ReadDecalsConfigurationFile`：解析 `roadpainter_decals.json`。
  - `LoadAssetsMaterials`：加载地图资产并检查道路网格。
  - `GetAssetsPathFromPackage`：读取包配置文件。
- **依赖模块**：
  - `FileHelpers`：保存包。
  - `JsonSerializer`：解析 JSON 配置文件。
  - `OpenDrive`：处理 `.xodr` 文件。

### 4.2 MoveAssetsCommandlet
- **核心类**：`UMoveAssetsCommandlet`
- **关键函数**：
  - `MoveAssetsFromMapForSemanticSegmentation`：加载资产并按语义分类。
  - `MoveFiles`：执行资产重命名和移动。
  - `ParseParams`：解析命令行参数。
- **依赖模块**：
  - `AssetTools`：资产重命名。
  - `SSTags`：语义标签定义。

### 4.3 PrepareAssetsForCookingCommandlet
- **核心类**：`UPrepareAssetsForCookingCommandlet`
- **关键函数**：
  - `PrepareMapsForCooking`：处理单张和分块地图。
  - `SpawnMeshesToWorld`：生成网格并应用材质。
  - `SaveWorld`：保存地图并处理 OpenDrive 数据。
  - `GenerateMapPathsFile`：生成路径文件。
- **依赖模块**：
  - `AssetRegistry`：资产管理。
  - `FileHelpers`：保存包。
  - `JsonSerializer`：解析 JSON 配置文件。
  - `LargeMapManager`：管理分块地图。

---

## 5. 错误处理和调试建议

### 5.1 常见问题
- **文件缺失**：
  - 问题：`TilesInfo.txt`、`roadpainter_decals.json` 或 `.xodr` 文件缺失。
  - 解决：检查文件路径，确保文件存在且格式正确。
- **路径错误**：
  - 问题：包配置文件或地图路径无效。
  - 解决：验证 `PackageName.Package.json` 和地图路径，检查日志中的错误信息。
- **资产未加载**：
  - 问题：地图或道具资产未正确加载。
  - 解决：确认资产路径和命名规则，检查 `UObjectLibrary` 加载日志。
- **保存失败**：
  - 问题：包保存失败或被覆盖。
  - 解决：检查目标路径是否可写，备份现有包。

### 5.2 调试技巧
- **启用日志**：所有 Commandlet 启用 `LogToConsole=true`，检查控制台输出以定位错误。
- **分步验证**：
  - 运行单个 Commandlet（如仅 `LoadAssetMaterialsCommandlet`），检查中间结果。
  - 验证配置文件格式和内容。
- **检查资产命名**：确保资产名称包含正确的语义标签（如 `R_ROAD1`）。
- **使用编辑器调试**：在编辑器中手动加载地图，检查资产和材质是否正确应用。
