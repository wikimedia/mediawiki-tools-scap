<template>
	You are not authorized to use SpiderPig
</template>

<script>
import { defineComponent, onMounted } from 'vue';
import { useRouter } from 'vue-router';
import useApi from '../api';

export default defineComponent( {
	name: 'NotAuthorized',
	setup() {
		const api = useApi();
		const router = useRouter();

		async function verifyUnauthorized() {
			const resp = await api.whoami();

			// Move the user to a more useful page if we can see
			// that they're authorized now.
			if ( resp.isAuthorized === true ) {
				router.push( '/' );
			}

		}

		onMounted( () => {
			verifyUnauthorized();
		} );

		return {
		};
	}
} );
</script>
