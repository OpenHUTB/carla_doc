<template>
    <div>
        <el-form size="mini" ref="form" :model="form" label-width="80px">
            <div style="display: flex;">
                <el-form-item label="车辆名称" style="margin-right: -2em;">
                    <el-input v-model="form.chedaoshu"></el-input>
                </el-form-item>
                <el-form-item label="种类" style="margin-right: -2em;">
                    <el-select v-model="form.leixing" placeholder="小轿车">
                        <el-option label="小轿车" value="gelilan"></el-option>
                        <el-option label="货车" value="weizhi"></el-option>
                    </el-select>
                </el-form-item>
                <el-form-item label="数量">
                    <el-input v-model="form.chedaoshu"></el-input>
                </el-form-item>
            </div>

            <span style="color: rgb(94, 94, 94);"> 车辆轨迹</span>
            <el-form-item label="起点路段" style="margin-right: -2em;">
                <el-select v-model="form.leixing" placeholder="手动选点">
                    <el-option label="小轿车" value="gelilan"></el-option>
                    <el-option label="货车" value="weizhi"></el-option>
                </el-select>
            </el-form-item>
            <el-form-item label="终点路段" style="margin-right: -2em;">
                <el-select v-model="form.leixing" placeholder="手动选点">
                    <el-option label="小轿车" value="gelilan"></el-option>
                    <el-option label="货车" value="weizhi"></el-option>
                </el-select>
            </el-form-item>
            <el-form-item label="速度" style="margin-right: -2em;">
                <el-select v-model="form.leixing" placeholder="随机生成">
                    <el-option label="小轿车" value="gelilan"></el-option>
                    <el-option label="货车" value="weizhi"></el-option>
                </el-select>
            </el-form-item>


        </el-form>
    </div>
</template>
<script>
export default {
    data() {
        return {
            form: {
                chedaoshu: '',
                feijidongchekuandu: '',
                jifeigelikuandu: '',
                leixing: '',
                delivery: false,
                type: [],
                resource: '',
                desc: ''
            }
        }
    },
    methods: {
        onSubmit() {
            console.log('submit!');
        }
    }
}
</script>