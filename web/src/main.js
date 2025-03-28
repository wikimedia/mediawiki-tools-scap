import { createApp } from 'vue';
import { createPinia } from 'pinia';
import router from './router';
import App from './App.vue';
import './assets/reset.css';

// Vuetify
import 'vuetify/styles';
import { createVuetify } from 'vuetify';
import * as components from 'vuetify/components';
import * as directives from 'vuetify/directives';

const vuetify = createVuetify( {
	components,
	directives
} );

createApp( App )
	.use( router )
	.use( createPinia() )
	.use( vuetify )
	.mount( '#app' );
