function v = CR_EULER_FREE_NONE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 20);
  end
  v = vInitialized;
end
