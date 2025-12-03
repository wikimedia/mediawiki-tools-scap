<template>
	<sp-job-page>
		<template #new-job>
			<sp-train />
		</template>
	</sp-job-page>
</template>

<script type="ts">
import { defineComponent } from 'vue';
import SpJobPage from './JobPage.vue';
import SpTrain from './Train.vue';

export default defineComponent( {
	name: 'TrainPage',
	components: {
		SpJobPage,
		SpTrain
	}
} );
</script>
