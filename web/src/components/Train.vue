<template>
	<div id="train">
		<div>
			<cdx-button @click="startTrain">
				Run scap train
			</cdx-button>
		</div>
	</div>
</template>

<script>
import useApi from '../api';
import { CdxButton } from '@wikimedia/codex';

export default {
	name: 'SpTrain',

	components: {
		CdxButton
	},

	data() {
		return {
			api: null
		};
	},

	methods: {
		async startTrain() {
			await this.api.startTrain();
		}
	},

	mounted() {
		this.api = useApi();
	}
};

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
