<template>
	<div id="train">
		<div>
			<cdx-button @click="startTrain">
				Run scap train
			</cdx-button>
		</div>
	</div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import useApi from '../api';
import { CdxButton } from '@wikimedia/codex';

export default defineComponent( {
	name: 'SpTrain',

	components: {
		CdxButton
	},

	setup() {
		// Pinia store.
		const api = useApi();

		async function startTrain() {
			await api.startTrain();
		}

		return {
			startTrain
		};
	}
} );

</script>

<style lang="less" scoped>
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

#train {
	display: inline-flex;
	flex-direction: column;
	border-style: solid;
	border-radius: 10px;
	border-width: @border-width-base;
	padding: @spacing-30;
	margin: @spacing-30;
}
</style>
