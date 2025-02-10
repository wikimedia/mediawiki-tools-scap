<template>
	<h1>Administrative Operations</h1>
	<div v-if="isAdmin">
		<ul>
			<li>
				<button
					class="cdx-button cdx-button--action-destructive"
					@click="logoutAll">
					Log out all users (including yourself)
				</button>
			</li>
			<li>Freeze/unfreeze deployments (placeholder)</li>
			<li>Ban/unban users (placeholder)</li>
		</ul>
	</div>
	<div v-else>
		Unauthorized
	</div>
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
