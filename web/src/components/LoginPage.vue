<template>
	<form @submit.prevent="login">
		<cdx-field :status="status" :messages="messages">
			<template #label>
				Please Login
			</template>

			<template #description>
				Login in with the username that the
				<code>spiderpig-apiserver</code> process is running under.
			</template>

			<cdx-label input-id="username">
				Username
			</cdx-label>

			<cdx-text-input id="username" v-model="username" />

			<cdx-label input-id="password">
				One time password
				<template #description>
					Run <code>scap spiderpig-otp</code> to generate a password.
				</template>
			</cdx-label>

			<cdx-text-input
				id="password"
				v-model="password"
				input-type="password"
			/>

			<br>

			<cdx-button
				action="progressive"
				weight="primary"
				:disabled="loginButtonDisabled"
				@click="login"
			>
				Login
			</cdx-button>
		</cdx-field>
	</form>
</template>

<script lang="ts">
import { defineComponent, ref, computed, watch } from 'vue';
import { CdxButton, CdxField, CdxTextInput, CdxLabel } from '@wikimedia/codex';
import useApi from '../api';
import { useRouter, useRoute } from 'vue-router';

export default defineComponent( {
	components: {
		CdxButton,
		CdxField,
		CdxTextInput,
		CdxLabel
	},
	setup() {
		// Pinia store and router.
		const api = useApi();
		const router = useRouter();
		const route = useRoute();

		// Reactive data properties.
		const status = ref( 'default' );
		const username = ref( '' );
		const password = ref( '' );
		const messages = ref( {} );
		const loginButtonDisabled = computed( () => username.value.trim() === '' || password.value.trim() === '' );

		// Methods.
		const login = async () => {
			const trimmedUsername = username.value.trim();
			const trimmedPassword = password.value.trim();
			const res = await api.login( trimmedUsername, trimmedPassword );

			if ( res !== true ) {
				status.value = 'error';
				messages.value.error = res.statusText;
				return;
			}

			// Login was successful
			router.push( route.query.redirect || '/' );
		};

		const clearErrorStatus = () => {
			if ( status.value === 'error' ) {
				status.value = 'default';
			}
		};

		watch( [ username, password ], clearErrorStatus );

		return {
			status,
			username,
			password,
			messages,
			loginButtonDisabled,
			login
		};
	}
} );
</script>
