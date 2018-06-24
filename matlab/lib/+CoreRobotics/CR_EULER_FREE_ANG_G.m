function v = CR_EULER_FREE_ANG_G()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 25);
  end
  v = vInitialized;
end
