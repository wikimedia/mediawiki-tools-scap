<template>
    <p>Login in with the username that the <code>spiderpig-apiserver</code> process is running under.</p>
    <p>Run <code>scap spiderpig-otp</code> to generate a password.</p>
    <div>
        <div>Login <input ref="username" @input="enableLoginButton" /></div>
        <div>One time password <input ref="password" @input="enableLoginButton" @keyup.enter="login" /></div>
        <button ref="loginButton" @click="login" disabled>Login</button>
    </div>
    <p v-if="status">{{ status }}</p>
</template>

<script>
import { login } from '../common'

export default {
    data() {
        return {
            status: null
        }
    },
    methods: {
        enableLoginButton() {
            const username = this.$refs.username.value.trim();
            const password = this.$refs.password.value.trim();

            this.$refs.loginButton.disabled = (username == "" || password == "")
        },
        async login() {
            const username = this.$refs.username.value.trim();
            const password = this.$refs.password.value.trim();

            const res = await login(username, password)

            if (res !== true) {
                this.status = res.statusText
                return
            }

            // Login was successful
            this.$router.push(this.$route.query.redirect || "/")
        }
    }

}
</script>

<style scoped></style>