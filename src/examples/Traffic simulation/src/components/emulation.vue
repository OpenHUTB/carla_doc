<template>
    <div>
        <div class="background-div">
            联合虚拟引擎仿真模拟实验
        </div>
        <div>
            <video controls loop autoplay style="width: 100%;">
                <source src="../assets/20231206_191951.mp4" type="video/mp4">

            </video>

        </div>
        <div class="background-div">
            岳麓大道-桐梓坡路口
        </div>
        <div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">摄像头配置</span>
            </div>
            <div class="container">
                <el-descriptions style="background-color: rgba(245, 245, 220, 0);" :column="1">

                    <el-descriptions-item label="设备序列号"><el-input v-model="input1" placeholder="154287953"
                            size="mini"></el-input>
                    </el-descriptions-item>
                    <el-descriptions-item label="IP地址"><el-input v-model="input1" placeholder="192.168.2.1"
                            size="mini"></el-input>
                    </el-descriptions-item>
                    <el-descriptions-item label="端口"><el-input v-model="input1" placeholder="8000" size="mini"></el-input>
                    </el-descriptions-item>
                    <el-descriptions-item label="子网掩码"><el-input v-model="input1" placeholder="255.255.255.0"
                            size="mini"></el-input>
                    </el-descriptions-item>
                    <el-descriptions-item label="网关"><el-input v-model="input1" placeholder="192.168.1.1"
                            size="mini"></el-input>
                    </el-descriptions-item> <el-descriptions-item label="管理员密码"><el-input v-model="input1"
                            placeholder="*******" size="mini"></el-input>
                    </el-descriptions-item>

                </el-descriptions>


            </div>

            <div>
                <el-tag :key="tag" v-for="tag in dynamicTags" closable :disable-transitions="false"
                    @close="handleClose(tag)">
                    {{ tag }}
                </el-tag>
                <el-input class="input-new-tag" v-if="inputVisible" v-model="inputValue" ref="saveTagInput" size="small"
                    @keyup.enter.native="handleInputConfirm" @blur="handleInputConfirm">
                </el-input>
                <el-button v-else class="button-new-tag" size="small" @click="showInput">激活摄像头</el-button>


            </div>

            <div class="container">
                <div class="left">
                    <div style=" width: 100%;height: 0; padding-bottom: 100%;background-color: antiquewhite;">

                        <video src="../assets/01.mp4" autoplay loop controls style="width: 100%;height: 1;"></video>
                    </div>
                    <div style=" width: 100%;height: 0; padding-bottom: 100%;background-color: antiquewhite;">


                        <video src="../assets/02.mp4" autoplay controls loop style="width: 100%;height: 1;"></video>

                    </div>

                </div>
                <div></div>
                <div class="center">
                    <div style=" width: 100%;height: 0; padding-bottom: 100%;background-color: antiquewhite;">

                        <video src="../assets/3.mp4" autoplay loop controls style="width: 100%;height: 1;"></video>
                    </div>
                    <div style=" width: 100%;height: 0; padding-bottom: 100%;background-color: antiquewhite;">


                        <video src="../assets/4.mp4" autoplay controls loop style="width: 100%;height: 1;"></video>

                    </div>

                </div>
                <div class="right">
                    <div style=" width: 100%;height: 0; padding-bottom: 100%;background-color: antiquewhite;">

                        <video src="../assets/5.mp4" autoplay loop controls style="width: 100%;height: 1;"></video>
                    </div>
                    <div style=" width: 100%;height: 0; padding-bottom: 100%;background-color: antiquewhite;">


                        <video src="../assets/6.mp4" autoplay controls loop style="width: 100%;height: 1;"></video>

                    </div>

                </div>
            </div>

        </div>
        <div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">红绿灯配时</span>
            </div>

            <div>
                <div class="block" style="display: flex; align-items: center;">
                    <span style="flex: 1;text-indent: 1.8em;">东西直行</span>
                    <el-slider v-model="value1" range :max="10" class="custom-slider" style="flex: 2;"></el-slider>
                </div>
                <div class="block" style="display: flex; align-items: center;">
                    <span style="flex: 1;text-indent: 1.8em;">东西转向</span>
                    <el-slider v-model="value02" range :max="10" class="custom-slider" style="flex: 2;"></el-slider>
                </div>

                <div class="block" style="display: flex; align-items: center;">
                    <span style="flex: 1;text-indent: 1.8em;">南北直行</span>
                    <el-slider v-model="value03" range :max="10" class="custom-slider" style="flex: 2;"></el-slider>
                </div>
                <div class="block" style="display: flex; align-items: center;">
                    <span style="flex: 1;text-indent: 1.8em;">南北转向</span>
                    <el-slider v-model="value04" range :max="10" class="custom-slider" style="flex: 2;"></el-slider>
                </div>
            </div>
        </div>
        <div>
            <div class="background-div02" style="">
                <span style="color: rgb(0, 170, 255);">|</span> <span style="color: rgb(172, 172, 172);">优化对比</span>
            </div>
            <Index/>

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
    height: 100%;
}

.center {
    flex: 1;
    background-color: rgb(238, 240, 128);
    height: 100%;
}

.right {
    flex: 1;
    background-color: lightcoral;
    height: 100%;
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

.custom-slider .el-slider__button {
    background-color: rgb(0, 255, 38);
    /* 设置滑块的颜色为红色 */
}

.custom-slider .el-slider__runway {
    background-color: rgb(255, 0, 0);
    /* 设置滑块条的底色为蓝色 */

}

.custom-slider .el-slider__bar {
    background-color: rgb(0, 255, 38);
    /* 设置滑块选择范围的颜色为红色 */
}

.custom-slider .el-slider__button-wrapper {
    display: none;
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
import Index from '@/components/Index.vue'


export default {
    name: 'road',
    name: 'VideoPlayer',
    components: {
        right,
        left,
        car,
        Index,
    },


    data() {
        return {

            value1: [0, 3],
            value02: [3, 6],
            value03: [6, 8],
            value04: [8, 10],
            dynamicTags: ['摄像头1', '摄像头北进口', '摄像头南进口'],
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
