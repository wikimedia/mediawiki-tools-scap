<template>
	<v-app>
		<navigation />

		<v-main color="surface-light">
			<RouterView />
		</v-main>
	</v-app>
</template>

<script lang="ts">
import '@wikimedia/codex/dist/codex.style.css';
import Navigation from './components/Navigation.vue';
import { VApp } from 'vuetify/components/VApp';
import { VMain } from 'vuetify/components/VMain';

export default {

	components: {
		Navigation,
		VApp,
		VMain
	}
};
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

main {
	.v-sheet {
		padding: @spacing-50 @spacing-100;
	}
}

strong {
	margin-left: 0.2em;
	margin-right: 0.2em;
}

</style>
