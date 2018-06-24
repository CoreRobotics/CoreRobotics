function v = CR_DH_FREE_NONE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 26);
  end
  v = vInitialized;
end
