<template>
    <div>
        <el-form size="mini" ref="form" :model="form" label-width="80px">
            <el-form-item label="车道数">
                <el-input v-model="form.chedaoshu"></el-input>
            </el-form-item>
            <el-checkbox-group v-model="form.type">
                <el-checkbox label="非机动车" name="type"> </el-checkbox><el-form-item label="宽度">
                    <el-input v-model="form.feijidongchekuandu"></el-input>
                </el-form-item>
                <el-checkbox label="机非隔离" name="type"> </el-checkbox><el-form-item label="宽度">
                    <el-input v-model="form.jifeigelikuandu"></el-input>
                </el-form-item>
                <el-form-item label="类型">
                    <el-select v-model="form.leixing" placeholder="请选择类型">
                        <el-option label="隔离栏" value="gelilan"></el-option>
                        <el-option label="未知" value="weizhi"></el-option>
                    </el-select>
                </el-form-item>

            </el-checkbox-group>



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