<template>
	<v-toolbar>
		<template #title>
			<h2>Administrative Operations</h2>
		</template>
	</v-toolbar>
	<v-sheet color="surface-light">
		<template v-if="isAdmin">
			<v-btn
				color="error"
				variant="tonal"
				@click="logoutAll"
			>
				Log out all users (including yourself)
			</v-btn>
			<br>
			<v-btn color="warning" variant="tonal">
				Freeze/unfreeze deployments (placeholder)
			</v-btn>
			<br>
			<v-btn color="warning" variant="tonal">
				Ban/unban users (placeholder)
			</v-btn>
		</template>
		<template v-else>
			<h3>Unauthorized</h3>
		</template>
	</v-sheet>
</template>

<script>
import useApi from '../api';
import { defineComponent, ref, onMounted } from 'vue';

export default defineComponent( {
	name: 'AdminPage',
	components: { },
	setup() {
		const api = useApi();

		const isAdmin = ref( false );

		async function setIsAdmin() {
			const resp = await api.whoami();
			isAdmin.value = resp.isAdmin;
		}

		async function logoutAll() {
			window.location = await api.logoutAll();
		}

		onMounted( () => setIsAdmin() );

		return {
			isAdmin,
			logoutAll
		};
	}
} );

</script>
