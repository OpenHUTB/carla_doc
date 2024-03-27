<template>
    <div>
        <div class="background-div02" style="">
            <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">仿真流量与真实流量对比</span>
        </div>
      <div
        class="echart"
        ref="mychart"
        id="mychart"
        :style="myChartStyle">
      </div>
    </div>
</template>

<style>
.background-div02 {
text-indent: 0.7em;
background-color: #343434E6;
}
</style>

<script>
import * as echarts from "echarts";


export default {
  data() {
    return {
      myChart: {},
      xData: ["0:00", "4:00", "8:00", "12:00", "16:00", "20:00"], //横坐标
      yyyData: [10,20,30],

      yData: [23, 24, 18, 25, 27, 28], 
      taskData: [20, 19, 19, 20, 20, 21], 
      myChartStyle: { float: "left", width: "100%", height: "400px" } //图表样式
    };
  },
  mounted() {
    this.initEcharts();
  },
  methods: {
    initEcharts() {
      const option = {
  title: {
    // text: 'Basic Radar Chart'
  },
  legend: {
    // data: ['Allocated Budget', 'Actual Spending']
    // data: ['Actual Spending']
  },
  radar: {
    // shape: 'circle',
    indicator: [
      { name: '流量', max: 6500 },
      { name: '车辆数', max: 16000 },
      { name: '密集度', max: 30000 },
      { name: '平均车头时距', max: 38000 },
      { name: '小时流率', max: 52000 },
      { name: '平均流速', max: 25000 }
    ]
  },
  series: [
    {
    //   name: 'Budget vs spending',
      type: 'radar',
      data: [
        // {
        //   value: [4200, 3000, 20000, 35000, 50000, 18000],
        //   name: 'Allocated Budget'
        // },
        {
          value: [5000, 14000, 28000, 26000, 42000, 21000],
        //   name: 'Actual Spending'
        }
      ],      
      areaStyle: {
        color: '#0F90E28A',
      },
      itemStyle: {
        color: '#47C5EEFF',
        borderColor: '#058AF3FF',
      },
    }
  ]
};
      const myChart = echarts.init(this.$refs.mychart);
      myChart.setOption(option);
      //随着屏幕大小调节图表
      window.addEventListener("resize", () => {
        myChart.resize();
      });
    }
  }
};
</script>