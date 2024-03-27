<template>
    <div>
        <div class="background-div">路口饱和度排行</div>
        <dv-capsule-chart :config="config" style="width:100%;height:120px;color: black;" />
    </div>
</template>

<style>
.background-div {
  background-image: url("../../assets/small.png");
  background-size: cover;
  background-position: center;
  text-indent: 3.5em;
}
</style>

<!-- //数据 -->
<script>
export default {
  name: 'HomeView',
  data() {
    return {
      config: {
        data: [
          {
            name: '秦华道-山姜北路',
            value: 167
          },
          {
            name: '华勤街道-岳麓大道',
            value: 123
          },
          {
            name: '普化路-其桥路',
            value: 98
          },
          {
            name: '永乐大道-麟山路',
            value: 75
          },
          {
            name: '北街坡路-心新林路',
            value: 66
          }
        ]
      }

    }
  }
}
</script>

