<template>
	<sp-job-page title="Deploy Service">
		<template #new-job="{ idle }">
			<sp-deploy-service :idle="idle" />
		</template>
	</sp-job-page>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import SpJobPage from './JobPage.vue';
import SpDeployService from './DeployService.vue';

export default defineComponent( {
	name: 'DeployServicePage',
	components: {
		SpJobPage,
		SpDeployService
	}
} );
</script>
