<template>
    <div class="interaction">
        <h2>Job {{ interaction.job_id }} is awaiting interaction</h2>
        <div class="prompt">{{ interaction.prompt }}</div>
        <cdx-button v-for="(code, choice) in interaction.choices" 
        :value="code"
        action="progressive"
        :weight="code===interaction.default ? 'primary' : 'normal'"
        @click="this.choiceSelected"
        >{{ choice }}</cdx-button>
    </div>
</template>

<script>
import useApi from '../api'

import { CdxButton } from '@wikimedia/codex';

export default {
    data() {
        return {
           api: null
        }
    },
    components: { CdxButton },
    props: ["interaction"],
    mounted() {
        this.api = useApi()
    },
    methods: {
        choiceSelected(e) {
            this.api.respondInteraction(this.interaction.job_id, this.interaction.id, e.currentTarget.value)
        }
    }
}
</script>

<style scoped>
.interaction {
    display: table;
    border-style: solid;
    border-width: 1px;
    border-radius: 10px;
    padding: 5px;
    margin: 5px;
}
.prompt {
    white-space: pre;
    font-family: 'Courier New', Courier, monospace;
    font-size: 15px;
}
</style>