<template>
  <div style="display: block">
    <div class="background-div">7日交通流量</div>
      <div
        class="echart"
        ref="mychart"
        id="mychart"
        :style="myChartStyle">
      </div>
  </div>
</template>

<style>
.el-dropdown-link {
  cursor: pointer;
  color: #409eff;
}
.el-icon-arrow-down {
  font-size: 12px;
}
.background-div {
  background-image: url("../../assets/small.png");
  background-size: cover;
  background-position: center;
  text-indent: 3.5em;
}
</style>

<script>
import * as echarts from "echarts";

export default {
  data() {
    return {
      xData: ["周一", "周二", "周三", "周四", "周五", "周六", "周日"], //横坐标
      yData: [23, 24, 18, 25, 27, 28, 25], //人数数据
      taskDate: [10, 11, 9, 17, 14, 13, 14],
      myChartStyle: { float: "left", width: "100%", height: "250px" }, //图表样式
    };
  },
  mounted() {
    this.initEcharts();
  },
  methods: {
    initEcharts() {
      // 多列柱状图
      const option = {
        xAxis: {
          data: this.xData,
        },
        // 图例
        legend: {
          data: ["机动车", "非机动车"],
          top: "0%",
        },
        yAxis: {},
        series: [
          {
            type: "bar", //形状为柱状图
            data: this.yData,
            name: "机动车", // legend属性
            label: {
              // 柱状图上方文本标签，默认展示数值信息
              show: true,
              position: "top",
            },
          },
          {
            type: "bar", //形状为柱状图
            data: this.taskDate,
            name: "非机动车", // legend属性
            label: {
              // 柱状图上方文本标签，默认展示数值信息
              show: true,
              position: "top",
            },
          },
        ],
      };
      const myChart = echarts.init(this.$refs.mychart); // 图标初始化
      myChart.setOption(option); // 渲染页面
      //随着屏幕大小调节图表
      window.addEventListener("resize", () => {
        myChart.resize();
      });
    },
  },
};
</script>
