<template>
    <div id="backport">
        <div>To run scap backport, enter one or more Gerrit change numbers separated by commas or spaces</div>
        <div>
            <input ref="backport_change_numbers" placeholder="Gerrit change numbers" @input="set_backport_button_status"
                @keyup.enter="start_backport" />
            <button ref="start_backport" disabled @click="start_backport">Run</button>
        </div>
    </div>
</template>

<script>
import { startBackport } from "../common.js"

export default {
    data() {
        return {}
    },
    methods: {
        set_backport_button_status() {
            const change_numbers = this.$refs.backport_change_numbers;
            const button = this.$refs.start_backport;

            if (change_numbers.value.trim() != "") {
                button.disabled = false;
            } else {
                button.disabled = true;
            }
        },
        async start_backport() {
            const change_numbers = this.$refs.backport_change_numbers;
            await startBackport(change_numbers.value.split(/[\s,]+/))
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