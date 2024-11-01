<template>
    <div id="app">
        <navigation id="app__nav" />

        <main id="app__content">
            <RouterView />
        </main>
    </div>
</template>

<script>
import '@wikimedia/codex/dist/codex.style.css'
import { CdxButton } from '@wikimedia/codex';
import Navigation from './components/Navigation.vue';

export default {
    data() {
        return {
            api: null
        }
    },

    components: {
        CdxButton,
        Navigation
    }
}
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

#app__nav {
    padding: @spacing-100;
}

#app__content {
    padding: @spacing-100;
}

</style>