function v = CR_DH_MODE_CLASSIC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 33);
  end
  v = vInitialized;
end
