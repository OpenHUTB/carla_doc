<template>
    <div class="home">
      <div class="main">
          <div class="YueLu"><YueLu /></div>
          <div class="Evaluation"><Evaluation /></div>
          <div class="TrafficFlow"><TrafficFlow/></div>
          <div class="TrafficDelay"><TrafficDelay/></div>
          <div class="Saturability"><Saturability/></div>
      </div>
    </div>
</template>
  
  <style>
  img {
    width: 100%;
    height: 100%;
    object-fit: cover;
  }

  .main {
    display: flex;
    flex-direction: column;
  }
  </style>
  
  <script>
  import YueLu from "./RightComponent/YueLu.vue";
  import Evaluation from "./RightComponent/Evaluation.vue";
  import TrafficFlow from "./RightComponent/TrafficFlow.vue";
  import TrafficDelay from "./RightComponent/TrafficDelay.vue";
  import Saturability from "./RightComponent/Saturability.vue";
  export default {
    components: {
      YueLu,
      Evaluation,
      TrafficFlow,
      TrafficDelay,
      Saturability,
    },
    methods: {},
  };
  </script>
  