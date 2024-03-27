<template>
  <div style="height: 100%; width: 100%">
    <div class="background-div">
            评价指标
    </div>
    <div style="height: 100%; width: 100%">
      <div style="position: relative">
        <el-dropdown
          style="
            border-radius: 100px;
            right: 0px;
            font-size: 10px;
            border: 1px solid #6996d7ff;
            background-color: #00000025;
            float: right;
            z-index: 10;
            padding: 10px 10px;
            position: absolute;
            height: 5px;

          "
          size="mini"
        >
          <span class="el-dropdown-link" style="height: 100%;display: flex;justify-content: center;align-items: center;">
            下拉菜单<i class="el-icon-arrow-down el-icon--right"></i>
          </span>
          <el-dropdown-menu type="card" slot="dropdown">
            <el-dropdown-item style="font-size: 10px;">东进口</el-dropdown-item>
            <el-dropdown-item style="font-size: 10px;">西进口</el-dropdown-item>
            <el-dropdown-item style="font-size: 10px;">北进口</el-dropdown-item>
            <el-dropdown-item style="font-size: 10px;">南进口</el-dropdown-item>
          </el-dropdown-menu>
        </el-dropdown>
      </div>

      <div style="flex: 1; width: 100%; height: 100%">
        <el-tabs type="border-card">
          <el-tab-pane label="实时">
            <el-table :data="tableData" 
                      style="width: 100%; font-size: 10px;" 
                      :header-cell-style="tableHeaderCellStyle"
                      :cell-style="tableCellStyle"
                      border
                      :row-style="rowStyle"
                      :fit=false
                      size="mini"
                      >
              <el-table-column
                width="100px"
                padding="0px 0px;"
                prop="name"
                label="指标名称"
              >
              </el-table-column>
              <el-table-column
                prop="left_1"
                label="1-左"
              >
              </el-table-column>
              <el-table-column
                prop="straight_2"
                label="2-直"
              >
              </el-table-column>
              <el-table-column
                prop="right_3"
                label="3-右"
              >
              </el-table-column>
              <el-table-column
                prop="left_4"
                label="4-左"
              >
              </el-table-column>
              <el-table-column
                prop="straight_5"
                label="5-直"
              >
              </el-table-column>
              <el-table-column
                prop="right_6"
                label="6-右"
              >
              </el-table-column>
            </el-table>
          </el-tab-pane>

          <el-tab-pane label="1分钟">
            <el-table :data="tableData" 
                      style="width: 100%; font-size: 10px;" 
                      :header-cell-style="tableHeaderCellStyle"
                      :cell-style="tableCellStyle"
                      border
                      :row-style="rowStyle"
                      :fit=false
                      size="mini"
                      >
              <el-table-column
                width="100px"
                padding="0px 0px;"
                prop="name"
                label="指标名称"
              >
              </el-table-column>
              <el-table-column
                prop="left_1"
                label="1-左"
              >
              </el-table-column>
              <el-table-column
                prop="straight_2"
                label="2-直"
              >
              </el-table-column>
              <el-table-column
                prop="right_3"
                label="3-右"
              >
              </el-table-column>
              <el-table-column
                prop="left_4"
                label="4-左"
              >
              </el-table-column>
              <el-table-column
                prop="straight_5"
                label="5-直"
              >
              </el-table-column>
              <el-table-column
                prop="right_6"
                label="6-右"
              >
              </el-table-column>
            </el-table>
          </el-tab-pane>

          >
          <el-tab-pane label="5分钟" style="font-size: 10px"
            >角色管理</el-tab-pane
          >
          <el-tab-pane label="1小时" style="font-size: 10px"
            >定时任务补偿</el-tab-pane
          >
        </el-tabs>
      </div>
    </div>
  </div>
</template>

<style lang="css" scoped>
label {
  font-size: 10px;
}

/* ::v-deep .el-tabs {
  width: 100px;
  height: 100%;
} */

.background-div {
    background-image: url('../../assets/small.png');
    background-size: cover;
    background-position: center;
    text-indent: 3.5em;
}

::v-deep .el-table tbody tr:hover > td {
   background-color: #1087E466 !important;
   color: #ffffff;
}


::v-deep .el-table--border .el-table__cell:first-child .cell{
  padding: 0;
}

::v-deep .el-table .cell {
  padding: 0;

}

::v-deep .el-tabs--border-card>.el-tabs__content {
  height: 180px;
  padding: 10px;
}

::v-deep .el-tabs__item {
  padding: 0 10px;
  height: auto;
  box-sizing: border-box;
  line-height: normal;
  display: inline-block;
  list-style: none;
  font-size: 10px;
  font-weight: 500;
  color: #303133;
  position: relative;
}

::v-deep .el-dropdown-item{
  font-size: 10px;
}

</style>

<script>
export default {
  data() {
    return {
      tableData: [
        {
          name:"停车数量",
          left_1:8,
          straight_2:5,
          right_3:10,
          left_4:6,
          straight_5:5,
          right_6:1
        },
        {
          name:"排队长度",
          left_1:16.4,
          straight_2:20.5,
          right_3:11.5,
          left_4:6.4,
          straight_5:5.8,
          right_6:14.8
        },
        {
          name:"平均速度",
          left_1:4.5,
          straight_2:9,
          right_3:9.2,
          left_4:5.8,
          straight_5:8.5,
          right_6:20.5
        },
        {
          name:"平均停车次数",
          left_1:6.8,
          straight_2:2,
          right_3:4,
          left_4:3.5,
          straight_5:7.5,
          right_6:0
        },
      ],
    };
  },
  methods:{
    /* 修改 table cell边框的背景色 */
    tableCellStyle () {
      return 'padding: 0px 0px !important;margin: 0;text-align:center;width: 1000px;'
    },
    tableHeaderCellStyle(){
      return 'color: #000000; fontSize: 10px;padding: 0px 0px !important;margin: 0;text-align:center;'
    },
    rowStyle(){
      return 'padding: 0px 0px;height:50px;'
    }
    
  },

};
</script>
