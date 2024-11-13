<template>
	<div id="app">
		<navigation id="app__nav" />

		<main id="app__content">
			<RouterView />
		</main>
	</div>
</template>

<script lang="ts">
import '@wikimedia/codex/dist/codex.style.css';
import Navigation from './components/Navigation.vue';

export default {

	components: {
		Navigation
	}
};
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
