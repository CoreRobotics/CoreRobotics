function v = CR_DH_FREE_THETA()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 31);
  end
  v = vInitialized;
end
