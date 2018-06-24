function v = CR_EULER_FREE_ANG_A()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 23);
  end
  v = vInitialized;
end
