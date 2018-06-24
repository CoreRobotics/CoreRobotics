function v = CR_DH_FREE_R()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 27);
  end
  v = vInitialized;
end
