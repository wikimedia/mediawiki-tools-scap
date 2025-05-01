<template>
	<sp-new-job-page title="MediaWiki Train">
		<template #new-job>
			<sp-train />
		</template>
	</sp-new-job-page>
</template>

<script type="ts">
import { defineComponent } from 'vue';
import SpNewJobPage from './NewJobPage.vue';
import SpTrain from './Train.vue';

export default defineComponent( {
	name: 'TrainPage',
	components: {
		SpNewJobPage,
		SpTrain
	}
} );
</script>
