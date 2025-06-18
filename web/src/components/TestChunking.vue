<template>
	<div>
		<cdx-field>
			<template #label>
				Interval (seconds)
			</template>
			<cdx-text-input
				v-model="interval"
				type="number" />
		</cdx-field>

		<cdx-field>
			<template #label>
				Size (bytes)
			</template>
			<cdx-text-input
				v-model="size"
				type="number"
			/>
		</cdx-field>
		<cdx-field>
			<template #label>
				Count (0 for all)
			</template>
			<cdx-text-input
				v-model="count"
				type="number"
			/>
		</cdx-field>
		<cdx-field>
			<template #label>
				Content Type
			</template>
			<cdx-text-input
				v-model="contentType"
				type="text"
			/>
		</cdx-field>
		<cdx-field>
			<template #label>
				Media Type
			</template>
			<cdx-text-input
				v-model="mediaType"
				type="text"
			/>
		</cdx-field>
		<cdx-button :disabled="fetchRunning" @click="fetchLogs">
			Fetch Logs
		</cdx-button>
		<br>

		<h2>Log entries</h2>
		<div id="log-output">
			<template v-for="( line, idx ) in logLines" :key="idx">
				<div>{{ line }}</div>
			</template>
		</div>
	</div>
</template>

<script lang="js">
/* eslint-disable no-console */
import { defineComponent, ref } from 'vue';
import { CdxButton, CdxTextInput, CdxField } from '@wikimedia/codex';

export default defineComponent( {
	name: 'TestChunking',
	components: {
		CdxButton,
		CdxTextInput,
		CdxField
	},

	setup() {
		const logLines = ref( [ ] );

		const interval = ref( 1 );
		const size = ref( 50 );
		const count = ref( 10 );
		const contentType = ref( '' );
		const mediaType = ref( '' );

		const fetchRunning = ref( false );

		async function* logRecordsProcessor( reader ) {
			const decoder = new TextDecoder();
			let buffer = '';

			while ( true ) {
				const { done, value } = await reader.read();

				if ( done ) {
					// Process any remaining data in the buffer
					if ( buffer ) {
						try {
							const json = JSON.parse( buffer );
							// Process the last JSON object
							yield json;
						} catch ( error ) {
							console.error( 'Error parsing final JSON:', error );
						}
					}
					break;
				}

				buffer += decoder.decode( value, { stream: true } );
				let newlineIndex;
				while ( ( newlineIndex = buffer.indexOf( '\n' ) ) !== -1 ) {
					const jsonString = buffer.slice( 0, newlineIndex );
					buffer = buffer.slice( newlineIndex + 1 );
					try {
						const json = JSON.parse( jsonString );
						// Process the parsed JSON object
						yield json;
					} catch ( error ) {
						console.error( 'Error parsing JSON:', error );
					}
				}
			}
		}

		function apiserverBaseURL() {
			return import.meta.env.VITE_APISERVER_URL ?
				import.meta.env.VITE_APISERVER_URL :
				window.location.origin;
		}

		function makeApiUrl( url ) {
			return new URL( url, apiserverBaseURL() );
		}

		async function fetchLogs() {
			// Reset
			logLines.value = [ ];
			fetchRunning.value = true;

			const params = new URLSearchParams(
				{
					interval: interval.value,
					size: size.value,
					count: count.value,
					contentType: contentType.value,
					mediaType: mediaType.value
				}
			);
			const queryString = params.toString();
			const fullUrl = makeApiUrl( `/api/test/log?${ queryString }` );

			const startTime = Date.now();

			console.log( `Fetching test logs from: ${ fullUrl }` );
			const resp = await fetch( fullUrl );
			console.log( resp );

			const fetchRespondedIn = Date.now() - startTime;
			logLines.value.push( `Fetch responded in ${ fetchRespondedIn } ms` );

			const reader = resp.body.getReader();
			let sawEOF = false;
			let lastTime = Date.now();

			for await ( const rec of logRecordsProcessor( reader ) ) {
				if ( rec.type === 'line' ) {
					logLines.value.push( `(+${ Date.now() - lastTime } ms): ${ rec.line } ` );
				} else if ( rec.type === 'EOF' ) {
					sawEOF = true;
					break;
				}
				lastTime = Date.now();
			}
			if ( sawEOF ) {
				logLines.value.push( '[End of job log]' );
			} else {
				logLines.value.push( 'Job log stream terminated prematurely' );
			}
			fetchRunning.value = false;
		}

		return {
			interval,
			size,
			count,
			contentType,
			mediaType,
			fetchLogs,
			logLines,
			fetchRunning
		};
	}
} );

</script>
