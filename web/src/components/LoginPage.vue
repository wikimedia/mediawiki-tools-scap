<template>
	<form @submit.prevent="login">
		<cdx-field :status="status" :messages="messages">
			<cdx-label input-id="password">
				Please enter using your one time password
				<template #description>
					To generate the one time password run:
					<code>ssh {{ user }}@{{ otpHost }} scap spiderpig-otp</code>
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
			>
				Login
			</cdx-button>
		</cdx-field>
	</form>
</template>

<script lang="ts">
import { defineComponent, ref, computed, watch, onMounted } from 'vue';
import { CdxButton, CdxField, CdxTextInput, CdxLabel, ValidationStatusType, ValidationMessages } from '@wikimedia/codex';
import useApi from '../api';
import { useRoute } from 'vue-router';

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
		const route = useRoute();
		const user = computed( () => api.authUser );
		const otpHost = ref( '...' );
		const redirectTarget = computed( () => {
			if ( route.query.redirect ) {
				// We're only using single strings as redirect targets in the route
				// definitions, so it's ok to use type coercion here for now
				return route.query.redirect as string;
			} else {
				return '/';
			}
		} );

		// Reactive data properties.
		const status = ref<ValidationStatusType>( 'default' );
		const messages = ref<ValidationMessages>( {} );
		const password = ref( '' );
		const loginButtonDisabled = computed( () => password.value.trim() === '' );

		// Methods.
		async function login() {
			const trimmedPassword = password.value.trim();
			const res = await api.login2( trimmedPassword );

			if ( res.code !== 'ok' ) {
				status.value = 'error';
				messages.value.error = res.message;
				return;
			}
			// Login was successful
			window.location.href = redirectTarget.value;
		}

		function clearErrorStatus() {
			if ( status.value === 'error' ) {
				status.value = 'default';
			}
		}

		async function setOtpHost() {
			const resp = await api.whoami();

			otpHost.value = resp.otpHost;
		}

		watch( [ password ], clearErrorStatus );

		onMounted( () => {
			setOtpHost();
		} );

		return {
			status,
			password,
			messages,
			loginButtonDisabled,
			login,
			user,
			otpHost
		};
	}
} );
</script>
