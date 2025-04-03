import { createApp } from 'vue';
import { createPinia } from 'pinia';
import router from './router';
import App from './App.vue';
import './assets/reset.css';

// Vuetify
import 'vuetify/styles';
import { createVuetify } from 'vuetify';

const vuetify = createVuetify();

createApp( App )
	.use( router )
	.use( createPinia() )
	.use( vuetify )
	.mount( '#app' );
