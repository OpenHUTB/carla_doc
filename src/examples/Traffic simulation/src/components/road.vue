<template>
    <div>
        <div class="background-div">
            道路生成
        </div>

        <div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">基本属性</span>
            </div>
            <div class="container">
                <el-descriptions style="background-color: rgba(245, 245, 220, 0);" :column="2">
                    <el-descriptions-item label="编号">508</el-descriptions-item>
                    <el-descriptions-item label="长度">0.12km</el-descriptions-item>
                    <el-descriptions-item label="名称"><el-input v-model="input1" placeholder="木华路一段" size="mini"></el-input>
                    </el-descriptions-item>
                    <el-descriptions-item label="所属道路">
                        <el-select v-model="value" placeholder="其他" size="mini">
                            <el-option v-for="item in options" :key="item.value" :label="item.label" :value="item.value">
                            </el-option>
                        </el-select>
                    </el-descriptions-item>
                    <el-descriptions-item label="车道宽度"><el-input v-model="input1" placeholder="3.75m"
                            size="mini"></el-input></el-descriptions-item>
                    <el-descriptions-item label="道路类型">
                        <el-select v-model="value" placeholder="城市道路" size="mini">
                            <el-option v-for="item in options" :key="item.value" :label="item.label" :value="item.value">
                            </el-option>
                        </el-select>
                    </el-descriptions-item>
                    <el-descriptions-item label="道路等级"><el-select v-model="value" placeholder="主干道" size="mini">
                            <el-option v-for="item in options" :key="item.value" :label="item.label" :value="item.value">
                            </el-option>
                        </el-select>
                    </el-descriptions-item>

                    <el-descriptions-item label="限速值"><el-input v-model="input1" placeholder="80 km/h"
                            size="mini"></el-input></el-descriptions-item>
                </el-descriptions>
            </div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">路段参数</span>
            </div>
            <div>
                <el-tabs v-model="activeName" type="border-card" @tab-click="handleClick" stretch="ture">
                    <el-tab-pane label="左车道" name="first">
                        <left />
                    </el-tab-pane>
                    <el-tab-pane label="右车道" name="second">
                        <right></right>
                    </el-tab-pane>

                </el-tabs>
            </div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">高程参数</span>
            </div>
            <div><el-radio-group v-model="radio">
                    <el-radio :label="3">高架/下穿</el-radio>
                    <el-radio :label="6">隧道</el-radio>
                    <el-radio :label="9">基本路段</el-radio>
                </el-radio-group>
            </div>
            <el-tag :key="tag" v-for="tag in dynamicTags" closable :disable-transitions="false" @close="handleClose(tag)">
                {{ tag }}
            </el-tag>
            <el-input class="input-new-tag" v-if="inputVisible" v-model="inputValue" ref="saveTagInput" size="small"
                @keyup.enter.native="handleInputConfirm" @blur="handleInputConfirm">
            </el-input>
            <el-button v-else class="button-new-tag" size="small" @click="showInput">添加道路</el-button>



        </div>
        <div class="background-div">
            车辆运动
        </div>

        <div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">车辆属性</span>
            </div>

            <div>
                <car></car>
            </div>

            <div class="container">

                <div>

                    <el-tag :key="tag" v-for="tag in dynamicTags02" closable :disable-transitions="false"
                        @close="handleClose(tag)">
                        {{ tag }}
                    </el-tag>
                    <el-input class="input-new-tag" v-if="inputVisible" v-model="inputValue" ref="saveTagInput" size="small"
                        @keyup.enter.native="handleInputConfirm" @blur="handleInputConfirm">
                    </el-input>
                    <el-button v-else class="button-new-tag" size="small" @click="showInput">添加车辆</el-button>
                </div>
            </div>




            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">模拟轨迹</span>
            </div>


            <div class="container">


                <div>
                    <video controls loop autoplay style="width: 100%;">
                        <source src="../assets/20231126_230535.mp4" type="video/mp4">

                    </video>
                </div>



            </div>

        </div>

    </div>
</template>
<style>
.container {
    display: flex;
}

.left {
    flex: 1;
    background-color: rgba(173, 216, 230, 0.944);
    height: 300px;
}

.center {
    flex: 1;
    background-color: rgb(238, 240, 128);
    height: 300px;
}

.right {
    flex: 1;
    background-color: lightcoral;
    height: 300px;
}

.background-div {
    background-image: url('../assets/small.png');
    background-size: cover;
    background-position: center;
    text-indent: 3.5em;
}

.background-div02 {

    text-indent: 0.7em;
    background-color: #343434E6;
}
</style>
<style>
.el-tag+.el-tag {
    margin-left: 10px;
}

.button-new-tag {
    margin-left: 10px;
    height: 32px;
    line-height: 30px;
    padding-top: 0;
    padding-bottom: 0;
}

.input-new-tag {
    width: 90px;
    margin-left: 10px;
    vertical-align: bottom;
}
</style>
<script>
import right from '@/components/right.vue'
import left from '@/components/left.vue'
import car from '@/components/car.vue'

export default {
    name: 'road',
    name: 'VideoPlayer',
    components: {
        right,
        left,
        car
    },


    data() {
        return {
            dynamicTags: ['五一大道路段2', '桐梓坡路段1', '桐梓坡路段2'],
            dynamicTags02: ['小轿车1', '小轿车', '货车'],
            inputVisible: false,
            inputValue: '',
            radio: '1',

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
        },

        handleClose(tag) {
            this.dynamicTags.splice(this.dynamicTags.indexOf(tag), 1);
        },

        showInput() {
            this.inputVisible = true;
            this.$nextTick(_ => {
                this.$refs.saveTagInput.$refs.input.focus();
            });
        },

        handleInputConfirm() {
            let inputValue = this.inputValue;
            if (inputValue) {
                this.dynamicTags.push(inputValue);
            }
            this.inputVisible = false;
            this.inputValue = '';
        },
    }
}

</script>
