<template>
	<cdx-field id="backport" class="backport">
		<template #label>
			<h2 class="backport__heading">
				Backport Patches
			</h2>
		</template>

		<template #description>
			To run scap backport, enter one or more Gerrit change numbers or URLs
		</template>

		<div class="backport__input">
			<cdx-multiselect-lookup
				v-model:input-chips="chips"
				v-model:selected="changeNumbers"
				class="backport__input__multiselect"
				:disabled="!idle"
				:menu-items="menuItems"
				:menu-config="menuConfig"
				placeholder="Enter change numbers or URLs"
				aria-label="Search for one or more Gerrit patches"
				@input="onInput"
			>
				<template #no-results>
					No results found.
				</template>
			</cdx-multiselect-lookup>

			<cdx-button
				:disabled="buttonDisabled"
				@click="startBackport"
			>
				Start Backport
			</cdx-button>

			<cdx-dialog
				v-model:open="alertDialogOpen"
				title="Failed to start backport"
				:use-close-button="alertDialogUseCloseButton"
			>
				{{ alertDialogText }}
			</cdx-dialog>
		</div>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed } from 'vue';
import { CdxButton, CdxDialog, CdxField, CdxMultiselectLookup } from '@wikimedia/codex';
import useApi from '../api';

export default {
	name: 'SpBackport',
	components: {
		CdxButton,
		CdxDialog,
		CdxField,
		CdxMultiselectLookup
	},

	props: {
		idle: {
			type: Boolean
		},
		initialChangeNumbers: {
			type: Array<string>,
			default: () => []
		}
	},

	setup( props ) {
		const api = useApi();
		const alertDialogOpen = ref( false );
		const alertDialogText = ref( '' );
		const alertDialogUseCloseButton = ref( true );
		const chips = ref( [] );
		const changeNumbers = ref( [] );
		const menuItems = ref( [] );

		// chips and changeNumbers must be in sync.
		changeNumbers.value = props.initialChangeNumbers;
		chips.value = props.initialChangeNumbers.map( ( id: string ) => ( { value: id } ) );

		const menuConfig = {
			boldLabel: true,
			visibleItemLimit: 1
		};

		const buttonDisabled = computed( () => {
			if ( changeNumbers.value.length > 0 ) {
				return false;
			} else {
				return true;
			}
		} );

		async function startBackport() {
			try {
				const res = await api.startBackport( changeNumbers.value );
				localStorage.setItem( 'spiderpig-bp-job', res.id );
				chips.value = [];
				changeNumbers.value = [];
				if ( Notification.permission === 'default' ) {
					await Notification.requestPermission();
				}
			} catch ( error ) {
				alertDialogOpen.value = true;
				alertDialogText.value = error.respJson.detail.message || error.message;
			}
		}

		async function fetchResults( changeId:string ) {
			return await api.searchPatch( changeId );
		}

		async function onInput( value: string ) {
			if ( !value ) {
				menuItems.value = [];
				return;
			}

			const data = await fetchResults( value );

			const results = data.map( ( item ) => {
				// eslint-disable-next-line no-underscore-dangle
				const changeNumber = String( item._number );

				return {
					label: changeNumber, // the Gerrit patch number, ex. 1084887
					description: item.subject, // The commit subject line
					value: changeNumber // the Gerrit patch number, ex. 1084887
				};
			} );

			menuItems.value = results;
		}

		return {
			alertDialogOpen,
			alertDialogText,
			alertDialogUseCloseButton,
			buttonDisabled,
			chips,
			changeNumbers,
			menuItems,
			menuConfig,
			startBackport,
			onInput
		};
	}
};
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.backport {
	&.cdx-field {
		margin-bottom: @spacing-150;
	}

	&__heading {
		font-size: @font-size-x-large;
	}

	&__input {
		display: flex;

		&__multiselect {
			flex: 1 0 auto;
		}

		.cdx-chip-input {
			border-top-right-radius: 0px;
			border-bottom-right-radius: 0px;
		}

		.cdx-button {
			border-top-left-radius: 0px;
			border-bottom-left-radius: 0px;
		}
	}
}

</style>
