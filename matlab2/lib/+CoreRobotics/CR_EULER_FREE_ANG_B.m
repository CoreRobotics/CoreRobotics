function v = CR_EULER_FREE_ANG_B()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 26);
  end
  v = vInitialized;
end
