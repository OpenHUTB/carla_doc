<template>
  <div id="app">

    <div>

      <el-header
        style="display: flex; justify-content: center; align-items: center;
        font-size: x-large;height: 50px;margin-top: 3px;">湘江实验室智慧城市
      </el-header>

      <el-container style="height: 100vh;">
        <el-aside style="height: 100%; width: 20%">
          <el-container>
            <el-header style="background-color: rgba(255, 228, 196, 0);max-height: 3em;padding: 0;">
              <el-tabs v-model="activeName" type="border-card" @tab-click="handleClick">
                <el-tab-pane label="道路编辑" name="first">
                  <road />
                </el-tab-pane>
                <el-tab-pane label="联合仿真" name="second">
                  <emulation></emulation>
                </el-tab-pane>
              </el-tabs>
            </el-header>
            <el-main style=""></el-main>
          </el-container>
        </el-aside>

        <el-main style="height: 100vh-100px;"></el-main>

        <el-aside style="height: 100%;width: 20%; ">
          <el-container style=" ">
            <el-header style="background-color: rgba(255, 228, 196, 0);max-height: 3em;padding: 0; ">
              <el-tabs v-model="activeName" type="border-card" @tab-click="handleClick " style=" ">
                <el-tab-pane label="路口评价" name="first" style=" ">
                  <Crossing/>
                </el-tab-pane>
                <el-tab-pane label="路网评价" name="second" style=" ">
                  <RoadNetwork/>
                </el-tab-pane>
              </el-tabs>
            </el-header>
            <el-main style=""></el-main>
          </el-container>
        </el-aside>
      </el-container>

      <router-view />
    </div>


  </div>
</template>

<style>
html {
  zoom: 150%;
}

body {
  color: white;
  background-color: #0b1c2d;
  background-image: url(./assets/大标题_1684810761505.png),url(./assets/底图.png);
  background-size: 100% 100%;

/* 当内容高度大于图片高度时，背景图像的位置相对于viewport固定 */
background-attachment: fixed;
/* 让背景图基于容器大小伸缩 */
background-size: cover;

}

::v-deep el-tab-pane{
   
}


</style>
<script>
import road from '@/components/road.vue'
import emulation from '@/components/emulation.vue'
import Crossing from '@/components/Crossing.vue'
import RoadNetwork from '@/components/RoadNetwork.vue'
export default {
  name: 'app',
  components: {
    road,
    emulation,
    Crossing,
    RoadNetwork
  },
  data() {
    return {
      input1: '',


      activeName: 'first',

      options: [{
        value: '选项1',
        label: '黄金糕'
      }, {
        value: '选项2',
        label: '双皮奶'
      }, {
        value: '选项3',
        label: '蚵仔煎'
      }, {
        value: '选项4',
        label: '龙须面'
      }, {
        value: '选项5',
        label: '北京烤鸭'
      }],
      value: '',
      radio1: '上海',
      tags: [
        { name: '五一大道路段2', type: '' },
        { name: '桐梓坡路段1', type: '' },
        { name: '桐梓坡路段2', type: '' },
      ]
      ,
      tagss: [
        { name: '小轿车1', type: '' },
        { name: '小轿车', type: '' },
        { name: '货车', type: '' },
      ]
    }
  },
  methods: {
    handleClick(tab, event) {
      console.log(tab, event);
    }
  }
}

</script>