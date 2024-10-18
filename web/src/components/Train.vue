<template>
    <div id="train">
        <div>
            <CdxButton ref="start_train" @click="start_train">Run scap train</CdxButton>
        </div>
    </div>
</template>

<script>
import useApi from '../api'

import { CdxButton } from '@wikimedia/codex';

export default {
    components: {
        CdxButton
    },
    data() {
        return {
            api: null
        }
    },
    mounted() {
        this.api = useApi()
    },
    methods: {
        async start_train() {
            await this.api.startTrain()
        },
    }
}

</script>

<style scoped>
#train {
    display: inline-flex;
    flex-direction: column;
    border-style: solid;
    border-radius: 10px;
    border-width: 1px;
    padding: 5px;
    margin: 5px;
}
</style>