<template>
    <div id="backport">
        <div>To run scap backport, enter one or more Gerrit change numbers separated by commas or spaces</div>
        <div>
            <input ref="backport_change_numbers" placeholder="Gerrit change numbers" @input="set_backport_button_status"
                @keyup.enter="start_backport" />
            <CdxButton ref="start_backport" :disabled="startButtonDisabled" @click="start_backport">Run</CdxButton>
        </div>
    </div>
</template>

<script>
import useApi from '../api';

import { CdxButton } from '@wikimedia/codex';

export default {
    components: {
        CdxButton
    },
    data() {
        return {
            api: null,
            startButtonDisabled: true
        }
    },
    mounted() {
        this.api = useApi()
    },
    methods: {
        set_backport_button_status() {
            const change_numbers = this.$refs.backport_change_numbers;

            if (change_numbers.value.trim() != "") {
                this.startButtonDisabled = false;
            } else {
                this.startButtonDisabled = true;
            }
        },
        async start_backport() {
            const change_numbers = this.$refs.backport_change_numbers;
            await this.api.startBackport(change_numbers.value.split(/[\s,]+/))
            change_numbers.value = "";
        },
    }
}

</script>

<style scoped>
#backport {
    display: inline-flex;
    flex-direction: column;
    border-style: solid;
    border-radius: 10px;
    border-width: 1px;
    padding: 5px;
    margin: 5px;
}
</style>